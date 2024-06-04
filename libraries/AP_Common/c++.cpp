//
// C++ runtime support not provided by Arduino
//
// Note: use new/delete with caution.  The heap is small and
// easily fragmented.

#include <AP_HAL/AP_HAL.h>
#include <stdlib.h>


#if CONFIG_HAL_BOARD == HAL_BOARD_QURT

#include <new>

/*
  These are the heap access function exported by the SLPI DSP image for Qurt OS.
*/
extern "C" {
    void *fc_heap_alloc(size_t size);
    void fc_heap_free(void* ptr);
    size_t fc_heap_size(void);
    size_t fc_heap_usage(void);
}

void *operator new (size_t size, std::nothrow_t const &nothrow) noexcept
{
	if (size < 1) {
		size = 1;
	}

	return (fc_heap_alloc(size));
}

void *operator new[](size_t size, std::nothrow_t const &nothrow) noexcept
{
	if (size < 1) {
		size = 1;
	}

	return (fc_heap_alloc(size));
}

/*
  These variants are for new without std::nothrow. We don't want to ever
  use these from ardupilot code
 */
void *operator new (size_t size)
{
	if (size < 1) {
		size = 1;
	}

	return (fc_heap_alloc(size));
}


void *operator new[](size_t size)
{
	if (size < 1) {
		size = 1;
	}

	return (fc_heap_alloc(size));
}

/*
	Override delete to free up memory to correct heap
*/

// extern const AP_HAL::HAL& hal;

void operator delete (void *p) noexcept
{
	if (p) { fc_heap_free(p); }

    // DEV_PRINTF("Heap size: %u, heap usage: %u", fc_heap_size(), fc_heap_usage());
}

void operator delete[](void *ptr) noexcept
{
	if (ptr) { fc_heap_free(ptr); }
}

#else

/*
  globally override new and delete to ensure that we always start with
  zero memory. This ensures consistent behaviour.
 */
void * operator new(size_t size)
{
    if (size < 1) {
        size = 1;
    }
    return(calloc(size, 1));
}

void operator delete(void *p)
{
    if (p) free(p);
}

void * operator new[](size_t size)
{
    if (size < 1) {
        size = 1;
    }
    return(calloc(size, 1));
}

void operator delete[](void * ptr)
{
    if (ptr) free(ptr);
}

#endif
