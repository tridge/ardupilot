/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright (C) 2018 Adam Green
 */

#include <AP_HAL/AP_HAL.h>

#if AP_CRASHDUMP_ENABLED && defined(HAL_CRASH_SERIAL_PORT)

#include <CrashCatcher.h>
#include <hal.h>
#include <string.h>

#include "CrashDump.h"
#include "hwdef/common/watchdog.h"

#ifndef HAL_CRASH_SERIAL_PORT_BAUD
#define HAL_CRASH_SERIAL_PORT_BAUD 921600
#endif

#if !defined(USART_ISR_RXNE)
#define USART_ISR_RXNE USART_ISR_RXNE_RXFNE
#endif

static bool uart_initialised;
static CrashCatcherInfo crash_info;

static void print_string(const char *string);
static void wait_for_user_input(void);
static void dump_bytes(const uint8_t *memory, size_t element_count);
static void dump_byte_as_hex(uint8_t byte);
static void dump_hex_digit(uint8_t nibble);
static void dump_halfwords(const uint16_t *memory, size_t element_count);
static void dump_words(const uint32_t *memory, size_t element_count);

void crashdump_serial_start(const CrashCatcherInfo *info)
{
    crash_info = *info;

    print_string("\r\n\r\n");
    if (info->isBKPT) {
        print_string("BREAKPOINT");
    } else {
        print_string("CRASH");
    }
    print_string(" ENCOUNTERED\r\n"
                 "Enable logging and then press any key to start dump.\r\n");

    wait_for_user_input();
    print_string("\r\n");
}

static void print_string(const char *string)
{
    while (*string) {
        CrashCatcher_putc(*string++);
    }
}

static void wait_for_user_input(void)
{
    CrashCatcher_getc();
}

void crashdump_serial_write(const void *memory,
                            CrashCatcherElementSizes element_size,
                            size_t element_count)
{
    switch (element_size) {
    case CRASH_CATCHER_BYTE:
        dump_bytes(static_cast<const uint8_t *>(memory), element_count);
        break;
    case CRASH_CATCHER_HALFWORD:
        dump_halfwords(static_cast<const uint16_t *>(memory), element_count);
        break;
    case CRASH_CATCHER_WORD:
        dump_words(static_cast<const uint32_t *>(memory), element_count);
        break;
    }
    print_string("\r\n");
}

static void dump_bytes(const uint8_t *memory, size_t element_count)
{
    for (size_t i = 0; i < element_count; i++) {
        if (i != 0 && (i & 0xFU) == 0) {
            print_string("\r\n");
        }
        dump_byte_as_hex(*memory++);
    }
}

static void dump_byte_as_hex(uint8_t byte)
{
    dump_hex_digit(byte >> 4U);
    dump_hex_digit(byte & 0xFU);
}

static void dump_hex_digit(uint8_t nibble)
{
    static const char hex_to_ascii[] = "0123456789ABCDEF";
    CrashCatcher_putc(hex_to_ascii[nibble]);
}

static void dump_halfwords(const uint16_t *memory, size_t element_count)
{
    for (size_t i = 0; i < element_count; i++) {
        uint16_t value = *memory++;
        if (i != 0 && (i & 0x7U) == 0) {
            print_string("\r\n");
        }
        dump_bytes(reinterpret_cast<uint8_t *>(&value), sizeof(value));
    }
}

static void dump_words(const uint32_t *memory, size_t element_count)
{
    for (size_t i = 0; i < element_count; i++) {
        uint32_t value = *memory++;
        if (i != 0 && (i & 0x3U) == 0) {
            print_string("\r\n");
        }
        dump_bytes(reinterpret_cast<uint8_t *>(&value), sizeof(value));
    }
}

CrashCatcherReturnCodes crashdump_serial_end(CrashCatcherReturnCodes return_code)
{
    print_string("\r\nEnd of dump\r\n");
    if (return_code == CRASH_CATCHER_TRY_AGAIN && crash_info.isBKPT) {
        return CRASH_CATCHER_EXIT;
    }
    return return_code;
}

static void init_uart(void)
{
    USART_TypeDef *uart = HAL_CRASH_SERIAL_PORT;
    IRQ_DISABLE_HAL_CRASH_SERIAL_PORT();
    RCC_RESET_HAL_CRASH_SERIAL_PORT();

    uint32_t clock_divisor;
#if defined(STM32F7) || defined(STM32H7) || defined(STM32F3) || defined(STM32G4) || defined(STM32L4) || defined(STM32L4PLUS)
    clock_divisor = uint32_t((HAL_CRASH_SERIAL_PORT_CLOCK + HAL_CRASH_SERIAL_PORT_BAUD / 2U) /
                             HAL_CRASH_SERIAL_PORT_BAUD);
#else
#if STM32_HAS_USART6
    if (uart == USART1 || uart == USART6) {
#else
    if (uart == USART1) {
#endif
        clock_divisor = (STM32_PCLK2 + HAL_CRASH_SERIAL_PORT_BAUD / 2U) / HAL_CRASH_SERIAL_PORT_BAUD;
    } else {
        clock_divisor = (STM32_PCLK1 + HAL_CRASH_SERIAL_PORT_BAUD / 2U) / HAL_CRASH_SERIAL_PORT_BAUD;
    }
#endif

    uart->BRR = clock_divisor;

#if defined(STM32F7) || defined(STM32H7) || defined(STM32F3) || defined(STM32G4) || defined(STM32L4) || defined(STM32L4PLUS)
    uart->ICR = 0xFFFFFFFFU;
#else
    uart->SR = 0;
    (void)uart->SR;
    (void)uart->DR;
#endif

    uart->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
    uart_initialised = true;
}

int CrashCatcher_getc(void)
{
    if (!uart_initialised) {
        init_uart();
    }
    USART_TypeDef *uart = HAL_CRASH_SERIAL_PORT;
    static const char *wait_for_string = "dump_crash_log";
    uint8_t current_offset = 0;
    while (true) {
#if defined(STM32F7) || defined(STM32H7) || defined(STM32F3) || defined(STM32G4) || defined(STM32L4) || defined(STM32L4PLUS)
        while (!(USART_ISR_RXNE & uart->ISR)) {}
        const uint8_t character = uart->RDR;
#else
        while (!(USART_SR_RXNE & uart->SR)) {}
        const uint8_t character = uart->DR;
#endif
        if (character == wait_for_string[current_offset]) {
            current_offset++;
            if (current_offset == strlen(wait_for_string)) {
                return 0;
            }
        } else {
            current_offset = 0;
        }
    }
}

void CrashCatcher_putc(int character)
{
    if (!uart_initialised) {
        init_uart();
    }
    USART_TypeDef *uart = HAL_CRASH_SERIAL_PORT;
#if defined(STM32F7) || defined(STM32H7) || defined(STM32F3) || defined(STM32G4) || defined(STM32L4) || defined(STM32L4PLUS)
    uart->TDR = character & 0xFF;
#else
    uart->DR = character & 0xFF;
#endif
#if defined(STM32F7) || defined(STM32H7) || defined(STM32F3) || defined(STM32G4) || defined(STM32L4) || defined(STM32L4PLUS)
    while (!(USART_ISR_TC & uart->ISR)) {
#else
    while (!(USART_SR_TC & uart->SR)) {
#endif
        stm32_watchdog_pat();
    }
}

#endif // AP_CRASHDUMP_ENABLED && defined(HAL_CRASH_SERIAL_PORT)
