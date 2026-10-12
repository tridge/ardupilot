// SPDX-License-Identifier: GPL-3.0-or-later
// Freestanding probe helpers. Volatile accesses prevent recursive lowering to
// the very memory routines being implemented. The production SDK must measure
// and choose its own optimized helpers.

#[no_mangle]
pub unsafe extern "C" fn memcpy(dest: *mut u8, src: *const u8, size: usize) -> *mut u8 {
    for i in 0..size { dest.add(i).write_volatile(src.add(i).read_volatile()); }
    dest
}

#[no_mangle]
pub unsafe extern "C" fn memmove(dest: *mut u8, src: *const u8, size: usize) -> *mut u8 {
    if (dest as usize) <= src as usize {
        memcpy(dest, src, size)
    } else {
        for i in (0..size).rev() { dest.add(i).write_volatile(src.add(i).read_volatile()); }
        dest
    }
}

#[no_mangle]
pub unsafe extern "C" fn memset(dest: *mut u8, value: i32, size: usize) -> *mut u8 {
    for i in 0..size { dest.add(i).write_volatile(value as u8); }
    dest
}

#[no_mangle]
pub unsafe extern "C" fn memcmp(a: *const u8, b: *const u8, size: usize) -> i32 {
    for i in 0..size {
        let diff = a.add(i).read_volatile() as i32 - b.add(i).read_volatile() as i32;
        if diff != 0 { return diff; }
    }
    0
}
