// SPDX-License-Identifier: GPL-3.0-or-later
// Conservative freestanding helpers, as in the relocation probe. Volatile
// accesses prevent recursive lowering to libc. Benchmark before optimizing.

#[no_mangle]
pub unsafe extern "C" fn memcpy(destination: *mut u8, source: *const u8, size: usize) -> *mut u8 {
    for index in 0..size {
        destination
            .add(index)
            .write_volatile(source.add(index).read_volatile());
    }
    destination
}

#[no_mangle]
pub unsafe extern "C" fn memmove(destination: *mut u8, source: *const u8, size: usize) -> *mut u8 {
    if destination as usize <= source as usize {
        memcpy(destination, source, size)
    } else {
        for index in (0..size).rev() {
            destination
                .add(index)
                .write_volatile(source.add(index).read_volatile());
        }
        destination
    }
}

#[no_mangle]
pub unsafe extern "C" fn memset(destination: *mut u8, value: i32, size: usize) -> *mut u8 {
    for index in 0..size {
        destination.add(index).write_volatile(value as u8);
    }
    destination
}

#[no_mangle]
pub unsafe extern "C" fn memcmp(first: *const u8, second: *const u8, size: usize) -> i32 {
    for index in 0..size {
        let difference =
            first.add(index).read_volatile() as i32 - second.add(index).read_volatile() as i32;
        if difference != 0 {
            return difference;
        }
    }
    0
}

#[no_mangle]
pub unsafe extern "C" fn bcmp(first: *const u8, second: *const u8, size: usize) -> i32 {
    memcmp(first, second, size)
}
