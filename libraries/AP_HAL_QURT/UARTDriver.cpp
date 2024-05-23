
#include "UARTDriver.h"
#include <AP_Common/ExpandingString.h>

extern const AP_HAL::HAL& hal;

QURT::UARTDriver::UARTDriver(const char *name)
{ 
	if (strcmp(name, "/dev/console") == 0) {
		_is_console = true;
		HAP_PRINTF("UART console created");
	}
}

void QURT::UARTDriver::printf(const char *fmt, ...)
{
	if (_is_console) {
		va_list ap;
		char buf[300];
		va_start(ap, fmt);
		vsnprintf(buf, sizeof(buf), fmt, ap);
		va_end(ap);
		HAP_PRINTF(buf);
	}
}

/* QURT implementations of virtual methods */
void QURT::UARTDriver::_begin(uint32_t b, uint16_t rxS, uint16_t txS) {}
void QURT::UARTDriver::_end() {}
void QURT::UARTDriver::_flush() {}
bool QURT::UARTDriver::is_initialized() { return false; }
bool QURT::UARTDriver::tx_pending() { return false; }

uint32_t QURT::UARTDriver::_available() { return 0; }
uint32_t QURT::UARTDriver::txspace() { return 1; }
bool QURT::UARTDriver::_discard_input() { return false; }
size_t QURT::UARTDriver::_write(const uint8_t *buffer, size_t size)
{
    return size;
}
ssize_t QURT::UARTDriver::_read(uint8_t *buffer, uint16_t size)
{
    return 0;
}

