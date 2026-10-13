// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]

use core::alloc::{GlobalAlloc, Layout};
use core::ffi::c_void;
use core::fmt::{self, Write};
use core::marker::PhantomData;
use core::panic::PanicInfo;
use core::ptr;
use core::sync::atomic::{AtomicBool, AtomicPtr, Ordering};

pub mod api;
pub mod enums;
mod bindings;
mod borrowed;
mod file;
mod log;
mod imports;
mod libm;
mod strtof;
mod math;
mod memory;
mod parameter;
mod values;
pub use math::{Quaternion, Vector3f};
pub use borrowed::RcChannel;
pub use parameter::Parameter;
pub use log::LogValue;
pub use values::{Location, MissionItem, Vector2f};

/// Patched by the loader before entry; read-only for the module's lifetime.
#[repr(transparent)]
struct Import(core::cell::UnsafeCell<u32>);

// Only the loader writes, before any Rust code runs. No module mutation API.
unsafe impl Sync for Import {}

impl Import {
    const fn new() -> Self {
        Self(core::cell::UnsafeCell::new(u32::MAX))
    }

    fn slot(&self) -> u32 {
        // The compiler must not replace the externally patched slot with its
        // initial value, including under fat LTO.
        unsafe { self.0.get().read_volatile() }
    }
}

#[cfg(test)]
mod binding_tests;
#[cfg(test)]
mod borrowed_tests;
#[cfg(test)]
mod value_tests;
#[cfg(test)]
mod file_tests;
#[cfg(test)]
mod log_tests;
#[cfg(test)]
mod math_tests;
#[cfg(test)]
mod libm_tests;
#[cfg(test)]
mod strtof_tests;
#[cfg(test)]
mod parameter_tests;
#[cfg(test)]
mod singleton_tests;

static ENTERED: AtomicBool = AtomicBool::new(false);
static HOST: AtomicPtr<api::HostApi> = AtomicPtr::new(ptr::null_mut());
static CONTEXT: AtomicPtr<c_void> = AtomicPtr::new(ptr::null_mut());

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    Stopped,
    Invalid,
    NoMemory,
    Unavailable,
    /// Transient binding-lock contention. Sleep before retrying; propagating
    /// this out of start reports a module error rather than retrying for you.
    Busy,
    Host,
}

pub type Result<T = ()> = core::result::Result<T, Error>;

impl Error {
    fn status(self) -> i32 {
        match self {
            Self::Stopped => api::STATUS_STOPPED,
            Self::Invalid => api::STATUS_INVALID,
            Self::NoMemory => api::STATUS_NO_MEMORY,
            Self::Unavailable => api::STATUS_UNAVAILABLE,
            Self::Busy => api::STATUS_BUSY,
            Self::Host => api::STATUS_HOST_ERROR,
        }
    }
}

fn result(status: i32) -> Result {
    match status {
        api::STATUS_OK => Ok(()),
        api::STATUS_STOPPED => Err(Error::Stopped),
        api::STATUS_INVALID => Err(Error::Invalid),
        api::STATUS_NO_MEMORY => Err(Error::NoMemory),
        api::STATUS_UNAVAILABLE => Err(Error::Unavailable),
        api::STATUS_BUSY => Err(Error::Busy),
        _ => Err(Error::Host),
    }
}

/// One native thread owns a module context. It cannot be sent or shared with
/// other threads, and safe module code cannot manufacture a firmware context.
pub struct Context<'a> {
    api: &'a api::HostApi,
    raw: *mut c_void,
    _not_send_sync: PhantomData<*mut ()>,
}

impl Context<'_> {
    pub fn time_us(&self) -> u64 {
        // Entry validates all function pointers before creating the Context.
        unsafe { self.api.time_us.unwrap()(self.raw) }
    }

    pub fn stop_requested(&self) -> bool {
        unsafe { self.api.stop_requested.unwrap()(self.raw) != 0 }
    }

    /// A stop request interrupts sleep. Propagate the error with `?` so module
    /// locals are dropped before returning through the firmware trampoline.
    pub fn sleep_ms(&self, milliseconds: u32) -> Result {
        result(unsafe { self.api.sleep_ms.unwrap()(self.raw, milliseconds) })
    }

    /// MAVLink severity 0 (emergency) through 7 (debug). The host copies the
    /// message during this call; it must not retain a pointer into the module.
    pub fn text(&self, severity: u8, message: &str) -> Result {
        if severity > 7 || message.len() > u32::MAX as usize {
            return Err(Error::Invalid);
        }
        result(unsafe {
            self.api.text.unwrap()(self.raw, severity, message.as_ptr(), message.len() as u32)
        })
    }
}

/// Firmware-only entry adapter. Each separately loaded image may enter once.
///
/// # Safety
/// `api` must point to a readable version/size prefix (two u32 fields). If its
/// size advertises a complete HostApi, that whole object must be readable,
/// immutable and alive until the module thread has been joined. `context` must be the live
/// firmware context expected by every callback. Callbacks must obey the host
/// contract, never unwind, and never invoke module code or retain its pointers.
#[doc(hidden)]
pub unsafe fn enter(
    api: *const api::HostApi,
    context: *mut c_void,
    run: fn(&Context<'_>) -> Result,
) -> i32 {
    if api.is_null()
        || context.is_null()
        || (api as usize) % core::mem::align_of::<api::HostApi>() != 0
    {
        return api::STATUS_INVALID;
    }
    // Read only the prefix before forming a reference to the complete table.
    let prefix = api.cast::<u32>();
    if prefix.read() != api::ABI_VERSION
        || (prefix.add(1).read() as usize) < core::mem::size_of::<api::HostApi>()
    {
        return api::STATUS_INVALID;
    }
    if !(*api).is_valid() {
        return api::STATUS_INVALID;
    }
    if ENTERED.swap(true, Ordering::AcqRel) {
        return api::STATUS_INVALID;
    }
    CONTEXT.store(context, Ordering::Relaxed);
    HOST.store(api.cast_mut(), Ordering::Release);
    let module = Context {
        api: &*api,
        raw: context,
        _not_send_sync: PhantomData,
    };
    let status = match run(&module) {
        Ok(()) => api::STATUS_OK,
        Err(error) => error.status(),
    };
    HOST.store(ptr::null_mut(), Ordering::Release);
    status
}

/// Only `module!` should install this allocator. Firmware reclaims outstanding
/// allocations after joining, including deliberately forgotten Rust objects.
#[doc(hidden)]
pub struct Allocator;

unsafe impl GlobalAlloc for Allocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let host = HOST.load(Ordering::Acquire);
        if host.is_null() || layout.size() > u32::MAX as usize || layout.align() > u32::MAX as usize
        {
            return ptr::null_mut();
        }
        // Never panic or unwind from allocator callbacks.
        match (*host).allocate {
            Some(allocate) => allocate(
                CONTEXT.load(Ordering::Relaxed),
                layout.size() as u32,
                layout.align() as u32,
            ),
            None => ptr::null_mut(),
        }
    }

    unsafe fn dealloc(&self, pointer: *mut u8, _: Layout) {
        let host = HOST.load(Ordering::Acquire);
        if !host.is_null() {
            if let Some(deallocate) = (*host).deallocate {
                deallocate(CONTEXT.load(Ordering::Relaxed), pointer);
            }
        }
    }
}

struct PanicMessage {
    bytes: [u8; 192],
    length: usize,
}

impl Write for PanicMessage {
    fn write_str(&mut self, value: &str) -> fmt::Result {
        let count = value.len().min(self.bytes.len() - self.length);
        self.bytes[self.length..self.length + count].copy_from_slice(&value.as_bytes()[..count]);
        self.length += count;
        Ok(())
    }
}

/// Report the failure without allocating or unwinding. The host panic callback
/// marks this module failed and parks the thread indefinitely at low priority.
/// Its heap, image and stack must remain owned until reboot.
#[doc(hidden)]
pub fn panic(info: &PanicInfo<'_>) -> ! {
    let host = HOST.load(Ordering::Acquire);
    if !host.is_null() {
        let mut message = PanicMessage {
            bytes: [0; 192],
            length: 0,
        };
        // Formatting a user-supplied Display value can allocate or panic
        // recursively. Only copy a literal message and format primitive
        // location fields; never invoke arbitrary user formatting here.
        let _ = message.write_str(
            info.message()
                .as_str()
                .unwrap_or("Rust panic (formatted message omitted)"),
        );
        if let Some(location) = info.location() {
            let _ = write!(
                &mut message,
                " at {}:{}:{}",
                location.file(),
                location.line(),
                location.column()
            );
        }
        unsafe {
            if let Some(report) = (*host).panic {
                report(
                    CONTEXT.load(Ordering::Relaxed),
                    message.bytes.as_ptr(),
                    message.length as u32,
                );
            }
        }
    }
    // Only reachable before entry, or if firmware violates the non-returning
    // panic callback contract. Never unwind across native firmware frames.
    loop {
        core::hint::spin_loop();
    }
}

/// Export the single native start symbol and install the no_std allocator and
/// panic handler. The supplied function takes `&Context` and returns `Result`.
#[macro_export]
macro_rules! module {
    ($start:path) => {
        #[global_allocator]
        static AP_RUST_ALLOCATOR: $crate::Allocator = $crate::Allocator;

        #[panic_handler]
        fn ap_rust_panic(info: &core::panic::PanicInfo<'_>) -> ! {
            $crate::panic(info)
        }

        #[no_mangle]
        pub unsafe extern "C" fn start(
            api: *const $crate::api::HostApi,
            context: *mut core::ffi::c_void,
        ) -> i32 {
            $crate::enter(api, context, $start)
        }
    };
}
