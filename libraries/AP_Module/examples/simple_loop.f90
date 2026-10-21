! SPDX-License-Identifier: GPL-3.0-or-later
! SITL/Linux talk demo: the worker loop and calculation are Fortran.
function module_run(context) bind(C, name="ap_module_fortran_run") result(status)
    use, intrinsic :: iso_c_binding
    use ap_module
    implicit none
    type(c_ptr), value :: context
    integer(c_int32_t) :: status
    real(c_float) :: gyro(3), seconds, magnitude

    status = send_text(context, "Fortran ready")
    if (status /= module_ok) return
    do while (.not. stop_requested(context))
        status = get_gyro(context, gyro)
        if (status /= module_ok) return
        seconds = real(time_us(context), c_float) * 1.0e-6_c_float
        magnitude = sqrt(sum(gyro * gyro))
        status = send_text_floats(context, "Fortran tick %.1f gyro %.3f", [seconds, magnitude])
        if (status /= module_ok) return
        status = sleep_ms(context, 1000_c_int32_t)
        if (status == module_stopped) exit
        if (status /= module_ok) return
    end do
    status = module_ok
end function
