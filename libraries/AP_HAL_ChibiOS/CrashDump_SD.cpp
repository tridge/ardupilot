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
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include "CrashDump.h"

#if AP_CRASHDUMP_FATFS_ENABLED && (HAL_USE_SDC || \
    (HAL_USE_MMC_SPI && CRASHDUMP_SD_SPI_SUPPORTED_MCU))

#include <AP_Common/AP_FWVersion.h>
#include <AP_Math/crc.h>
#include <hal.h>

#include "hwdef/common/bouncebuffer.h"
#include "sdcard.h"
#include "SPIDevice.h"
#include "hwdef/common/stm32_util.h"
#include "hwdef/common/watchdog.h"

#include <ff.h>
#include <hal_mmcsd.h>
#if HAL_USE_SDC
#include <hal_sdc.h>
#endif
#include <stdio.h>
#include <string.h>

#if HAL_USE_SDC && (defined(STM32H7) || defined(STM32L4PLUS))
#define CRASHDUMP_SD_SPI 0
#define CRASHDUMP_SD_SDMMCV2 1
#define CRASHDUMP_SD_SDMMCV1 0
#define CRASHDUMP_SD_SDIOV1 0
#elif HAL_USE_SDC && (defined(STM32F7) || defined(STM32L4))
#define CRASHDUMP_SD_SPI 0
#define CRASHDUMP_SD_SDMMCV2 0
#define CRASHDUMP_SD_SDMMCV1 1
#define CRASHDUMP_SD_SDIOV1 0
#elif HAL_USE_SDC && defined(STM32F4)
#define CRASHDUMP_SD_SPI 0
#define CRASHDUMP_SD_SDMMCV2 0
#define CRASHDUMP_SD_SDMMCV1 0
#define CRASHDUMP_SD_SDIOV1 1
#elif HAL_USE_MMC_SPI && CRASHDUMP_SD_SPI_SUPPORTED_MCU
#define CRASHDUMP_SD_SPI 1
#define CRASHDUMP_SD_SDMMCV2 0
#define CRASHDUMP_SD_SDMMCV1 0
#define CRASHDUMP_SD_SDIOV1 0
#else
#error "SD crash dumps are not supported on this STM32 family"
#endif

/*
  SD crash dumps cannot use FatFs or ChibiOS synchronization from a fault
  handler. At boot a file is allocated and its physical sector extents are
  recorded. The fault handler then drives SDMMC directly in polling mode.
 */

#define CRASHDUMP_SD_MAX_EXTENTS 64U
#define CRASHDUMP_OVERHEAD (8U * 1024U)
#if CRASHDUMP_SD_SDMMCV2
#define SDMMC_ICR_ALL_FLAGS 0xFFFFFFFFU
#define SDMMC_DATA_ERROR_FLAGS (SDMMC_STA_DCRCFAIL | SDMMC_STA_DTIMEOUT | \
                                SDMMC_STA_TXUNDERR | SDMMC_STA_RXOVERR)
#elif CRASHDUMP_SD_SDMMCV1
// Some STM32F7 CMSIS headers omit the start-bit error definition.
#ifndef SDMMC_STA_STBITERR
#define SDMMC_STA_STBITERR (0x1UL << 9U)
#endif
#define SDMMC_ICR_ALL_FLAGS 0xFFFFFFFFU
#define SDMMC_DATA_ERROR_FLAGS (SDMMC_STA_DCRCFAIL | SDMMC_STA_DTIMEOUT | \
                                SDMMC_STA_TXUNDERR | SDMMC_STA_RXOVERR | \
                                SDMMC_STA_STBITERR)
#elif CRASHDUMP_SD_SDIOV1
#define SDIO_DATA_ERROR_FLAGS (SDIO_STA_DCRCFAIL | SDIO_STA_DTIMEOUT | \
                               SDIO_STA_TXUNDERR | SDIO_STA_RXOVERR | \
                               SDIO_STA_STBITERR)
#define CRASHDUMP_SDIO_ICR_ALL_FLAGS 0xFFFFFFFFU
#endif
#define CRASHDUMP_TRAILER_VERSION 1U

extern const AP_HAL::HAL& hal;

static constexpr uint8_t crashdump_trailer_magic[8] = {
    'A', 'P', 'C', 'D', 'U', 'M', 'P', 0
};
static constexpr char crashdump_reserved_path[] = "APM/CD_Reserved.DAT";
static constexpr char crashdump_published_path[] = "APM/CrashDump.DAT";

struct PACKED CrashDumpTrailer {
    uint8_t magic[sizeof(crashdump_trailer_magic)];
    uint16_t version;
    uint16_t size;
    uint32_t git_hash;
    uint32_t firmware_crc;
    uint32_t firmware_size;
    uint32_t trailer_crc;
    uint32_t dump_size;
};

static_assert(sizeof(CrashDumpTrailer) == 32U, "Unexpected crashdump trailer size");

extern "C" {
    extern const uint8_t __firmware_crc_start__;
    extern const uint8_t __firmware_crc_end__;
    extern const uint8_t __firmware_crc_ext_start__;
    extern const uint8_t __firmware_crc_ext_end__;
}

struct CrashDumpExtent {
    uint32_t start_sector;
    uint32_t sector_count;
};

static CrashDumpExtent *sd_extents;
static uint16_t sd_extent_count;
static uint32_t sd_total_sectors;
#if HAL_USE_SDC
static SDCDriver *sd_sdcp;
#elif CRASHDUMP_SD_SPI
extern MMCDriver MMCD1;
static MMCDriver *sd_mmcp;
static ChibiOS::SPIDevice *sd_spi_device;
static SPIDriver *sd_spip;
static struct bouncebuffer_t *sd_spi_bouncebuffer;
static ioline_t sd_spi_cs_line;
static uint32_t sd_spi_low_config1;
static uint32_t sd_spi_low_config2;
static uint32_t sd_spi_high_config1;
static uint32_t sd_spi_high_config2;
static bool sd_spi_block_addresses;
#endif
static uint8_t *sd_dma_buf;
static uint32_t sd_dma_buf_size;
static uint32_t accumulator_offset;
static uint32_t sd_write_offset;
static uint32_t sd_dump_size;
static uint32_t sd_firmware_crc;
static uint32_t sd_firmware_size;
static uint32_t sd_firmware_git_hash;
static uint32_t sd_dump_check_ms;
static bool sd_is_ready;
static bool sd_write_failed;

static uint32_t min_u32(uint32_t a, uint32_t b)
{
    return a < b ? a : b;
}

static uint32_t crashdump_sd_file_size()
{
    return HAL_CC_MEMORY_TOTAL_BYTES + CRASHDUMP_OVERHEAD;
}

enum class CrashDumpFileState : uint8_t {
    MISSING,
    EMPTY,
    COMPLETE,
    INCOMPLETE,
    IO_ERROR,
};

// Keep these values stable as they are printed during early boot diagnostics.
enum class CrashDumpDiagnostic : uint8_t {
    RESERVED_INCOMPLETE = 1,
    NO_SD_SPI_DEVICE = 2,
    SD_SPI_NOT_RUNNING = 3,
    NO_SD_SPI_BOUNCEBUFFER = 4,
    NO_SD_BOUNCEBUFFER = 5,
    SD_BOUNCEBUFFER_TOO_SMALL = 6,
    PUBLISH_RESERVED = 7,
    OPEN_RESERVED = 8,
    RECREATE_RESERVED = 9,
    INITIALISE_RESERVED = 10,
    SYNC_RESERVED = 11,
    INVALID_RESERVED_SIZE = 12,
    REOPEN_RESERVED = 13,
    EXTENT_WORK_ALLOCATION = 14,
    EXTENT_SEEK = 15,
    EXTENT_COUNT = 16,
    EXTENT_INVALID = 17,
    EXTENT_INCOMPLETE = 18,
    EXTENT_FINAL_ALLOCATION = 19,
    RESERVE_CREATE_ARMED = 20,
};

static bool unlink_if_exists(const char *path)
{
    const FRESULT result = f_unlink(path);
    return result == FR_OK || result == FR_NO_FILE;
}

static void report_diagnostic(CrashDumpDiagnostic code, uint32_t detail = 0)
{
    printf("CrashDumpSD: %u/%u\n", unsigned(code), unsigned(detail));
}

static bool init_failed(CrashDumpDiagnostic code, uint32_t detail = 0)
{
    report_diagnostic(code, detail);
    return false;
}

/* Validate a CrashCatcher dump and its completion trailer. */
static CrashDumpFileState get_dump_state(const char *path, uint32_t &dump_size)
{
    dump_size = 0;

    FIL fp;
    const FRESULT open_result = f_open(&fp, path, FA_READ);
    if (open_result == FR_NO_FILE) {
        return CrashDumpFileState::MISSING;
    }
    if (open_result != FR_OK) {
        return CrashDumpFileState::IO_ERROR;
    }

    const uint32_t file_size = f_size(&fp);
    if (file_size < 2U * MMCSD_BLOCK_SIZE ||
        (file_size % MMCSD_BLOCK_SIZE) != 0U) {
        f_close(&fp);
        return CrashDumpFileState::INCOMPLETE;
    }

    uint8_t signature[2];
    UINT bytes_read;
    if (f_lseek(&fp, 0) != FR_OK ||
        f_read(&fp, signature, sizeof(signature), &bytes_read) != FR_OK ||
        bytes_read != sizeof(signature)) {
        f_close(&fp);
        return CrashDumpFileState::IO_ERROR;
    }
    if (signature[0] != 0x63 || signature[1] != 0x43) {
        f_close(&fp);
        return CrashDumpFileState::EMPTY;
    }

    CrashDumpTrailer trailer;
    if (f_lseek(&fp, file_size - sizeof(trailer)) != FR_OK ||
        f_read(&fp, &trailer, sizeof(trailer), &bytes_read) != FR_OK ||
        bytes_read != sizeof(trailer)) {
        f_close(&fp);
        return CrashDumpFileState::IO_ERROR;
    }

    const uint32_t trailer_crc = trailer.trailer_crc;
    trailer.trailer_crc = 0;
    const bool valid_trailer =
        memcmp(trailer.magic, crashdump_trailer_magic, sizeof(trailer.magic)) == 0 &&
        trailer.version == CRASHDUMP_TRAILER_VERSION &&
        trailer.size == sizeof(trailer) &&
        trailer_crc == crc_crc32(0, reinterpret_cast<const uint8_t *>(&trailer),
                                 sizeof(trailer));
    const uint32_t max_size = file_size - MMCSD_BLOCK_SIZE;
    if (!valid_trailer || trailer.dump_size == 0 || trailer.dump_size > max_size) {
        f_close(&fp);
        return CrashDumpFileState::INCOMPLETE;
    }

    uint8_t padding[16];
    if (f_lseek(&fp, trailer.dump_size) != FR_OK ||
        f_read(&fp, padding, sizeof(padding), &bytes_read) != FR_OK ||
        bytes_read != sizeof(padding)) {
        f_close(&fp);
        return CrashDumpFileState::IO_ERROR;
    }
    f_close(&fp);
    for (uint8_t byte : padding) {
        if (byte != 0xFF) {
            return CrashDumpFileState::INCOMPLETE;
        }
    }

    dump_size = trailer.dump_size;
    return CrashDumpFileState::COMPLETE;
}

/*
  Publish a completed dump left in the reserved file by the fault handler.
  All directory operations happen at boot, never in the fault handler.
 */
static bool publish_crashdump(bool &reset_reserved)
{
    reset_reserved = false;
    uint32_t dump_size;
    const CrashDumpFileState state = get_dump_state(crashdump_reserved_path, dump_size);
    if (state == CrashDumpFileState::IO_ERROR) {
        return false;
    }
    if (state == CrashDumpFileState::INCOMPLETE) {
        reset_reserved = true;
        report_diagnostic(CrashDumpDiagnostic::RESERVED_INCOMPLETE);
        return true;
    }
    if (state == CrashDumpFileState::COMPLETE) {
        if (!unlink_if_exists(crashdump_published_path) ||
            f_rename(crashdump_reserved_path, crashdump_published_path) != FR_OK) {
            return false;
        }
    }
    return true;
}

static void calculate_firmware_identity()
{
    const uintptr_t start = reinterpret_cast<uintptr_t>(&__firmware_crc_start__);
    const uintptr_t end = reinterpret_cast<uintptr_t>(&__firmware_crc_end__);
    const uintptr_t ext_start = reinterpret_cast<uintptr_t>(&__firmware_crc_ext_start__);
    const uintptr_t ext_end = reinterpret_cast<uintptr_t>(&__firmware_crc_ext_end__);

    sd_firmware_crc = 0;
    sd_firmware_size = 0;
    const uintptr_t ranges[][2] = {
        {start, end},
        {ext_start, ext_end},
    };
    for (const auto &range : ranges) {
        uintptr_t address = range[0];
        while (address < range[1]) {
            const uint32_t size = min_u32(range[1] - address, 32U * 1024U);
            sd_firmware_crc = crc_crc32(sd_firmware_crc,
                                        reinterpret_cast<const uint8_t *>(address), size);
            sd_firmware_size += size;
            address += size;
            stm32_watchdog_pat();
        }
    }
    sd_firmware_git_hash = AP::fwversion().fw_hash;
}

/*
  Convert a file sector offset to an SD sector and return the number of
  physically contiguous sectors remaining in the extent.
 */
static bool sector_mapping(uint32_t sector_offset, uint32_t &sector, uint32_t &available)
{
    for (uint16_t i = 0; i < sd_extent_count; i++) {
        if (sector_offset < sd_extents[i].sector_count) {
            sector = sd_extents[i].start_sector + sector_offset;
            available = sd_extents[i].sector_count - sector_offset;
            return true;
        }
        sector_offset -= sd_extents[i].sector_count;
    }
    return false;
}

/* Build sector extents by walking the file one cluster at a time. */
static bool build_extent_list(FIL &fp, uint32_t num_sectors)
{
    CrashDumpExtent *const max_extents = NEW_NOTHROW CrashDumpExtent[CRASHDUMP_SD_MAX_EXTENTS];
    if (max_extents == nullptr) {
        return init_failed(CrashDumpDiagnostic::EXTENT_WORK_ALLOCATION);
    }

    FATFS *const fs = fp.obj.fs;
    uint16_t extent_count = 0;
    uint32_t total_sectors = 0;
    uint32_t sectors_remaining = num_sectors;
    while (sectors_remaining > 0) {
        const uint32_t sector_count = min_u32(fs->csize, sectors_remaining);
        // FatFs leaves fp.clust at the cluster containing seek_offset - 1.
        // Cluster-end seeks are sector-aligned and advance the chain linearly.
        const FSIZE_t seek_offset = FSIZE_t(total_sectors + sector_count) * MMCSD_BLOCK_SIZE;
        const FRESULT seek_result = f_lseek(&fp, seek_offset);
        if (seek_result != FR_OK || f_tell(&fp) != seek_offset) {
            delete[] max_extents;
            return init_failed(CrashDumpDiagnostic::EXTENT_SEEK, seek_result);
        }

        const uint32_t cluster = fp.clust;
        if (cluster < 2U || cluster >= fs->n_fatent) {
            delete[] max_extents;
            return init_failed(CrashDumpDiagnostic::EXTENT_INVALID);
        }
        const uint32_t start_sector = fs->database +
                                      (cluster - 2U) * fs->csize;
        if (extent_count > 0 &&
            max_extents[extent_count - 1U].start_sector +
            max_extents[extent_count - 1U].sector_count == start_sector) {
            max_extents[extent_count - 1U].sector_count += sector_count;
        } else {
            if (extent_count >= CRASHDUMP_SD_MAX_EXTENTS) {
                delete[] max_extents;
                return init_failed(CrashDumpDiagnostic::EXTENT_COUNT,
                                   CRASHDUMP_SD_MAX_EXTENTS);
            }
            max_extents[extent_count++] = {start_sector, sector_count};
        }
        total_sectors += sector_count;
        sectors_remaining -= sector_count;
        stm32_watchdog_pat();
    }
    if (extent_count == 0) {
        delete[] max_extents;
        return init_failed(CrashDumpDiagnostic::EXTENT_INCOMPLETE);
    }

    sd_extents = NEW_NOTHROW CrashDumpExtent[extent_count];
    if (sd_extents == nullptr) {
        delete[] max_extents;
        return init_failed(CrashDumpDiagnostic::EXTENT_FINAL_ALLOCATION);
    }
    memcpy(sd_extents, max_extents, extent_count * sizeof(*sd_extents));
    sd_extent_count = extent_count;
    sd_total_sectors = total_sectors;
    delete[] max_extents;
    return true;
}

#if CRASHDUMP_SD_SPI

#define CRASHDUMP_SPI_BYTE_TIMEOUT 100000U
#define CRASHDUMP_SPI_RESPONSE_BYTES 16U
#define CRASHDUMP_SPI_BUSY_BYTES 2000000U

static bool spi_exchange(uint8_t tx, uint8_t &rx)
{
    SPI_TypeDef *const spi = sd_spip->spi;
    uint32_t timeout = CRASHDUMP_SPI_BYTE_TIMEOUT;
#if defined(STM32H7)
    while ((spi->CR1 & SPI_CR1_CSTART) != 0U) {
        if (--timeout == 0U) {
            return false;
        }
    }
    spi->IFCR = 0xFFFFFFFFU;
    spi->CR1 |= SPI_CR1_CSTART;
    timeout = CRASHDUMP_SPI_BYTE_TIMEOUT;
    while ((spi->SR & SPI_SR_TXP) == 0U) {
        if (--timeout == 0U) {
            return false;
        }
    }
    *reinterpret_cast<volatile uint8_t *>(&spi->TXDR) = tx;
    timeout = CRASHDUMP_SPI_BYTE_TIMEOUT;
    while ((spi->SR & SPI_SR_RXP) == 0U) {
        if (--timeout == 0U) {
            return false;
        }
    }
    rx = *reinterpret_cast<volatile uint8_t *>(&spi->RXDR);
    spi->CR1 |= SPI_CR1_CSUSP;
#else
    while ((spi->SR & SPI_SR_TXE) == 0U) {
        if (--timeout == 0U) {
            return false;
        }
    }
    *reinterpret_cast<volatile uint8_t *>(&spi->DR) = tx;
    timeout = CRASHDUMP_SPI_BYTE_TIMEOUT;
    while ((spi->SR & SPI_SR_RXNE) == 0U) {
        if (--timeout == 0U) {
            return false;
        }
    }
    rx = *reinterpret_cast<volatile uint8_t *>(&spi->DR);
#endif
    return true;
}

static bool spi_send(uint8_t value)
{
    uint8_t ignored;
    return spi_exchange(value, ignored);
}

static bool spi_clock_bytes(uint32_t count)
{
    while (count-- > 0U) {
        if (!spi_send(0xFFU)) {
            return false;
        }
    }
    return true;
}

static void spi_select()
{
    palClearLine(sd_spi_cs_line);
}

static void spi_unselect()
{
    palSetLine(sd_spi_cs_line);
    (void)spi_send(0xFFU);
}

static bool spi_wait_idle()
{
    for (uint32_t i = 0; i < CRASHDUMP_SPI_BUSY_BYTES; i++) {
        uint8_t response;
        if (!spi_exchange(0xFFU, response)) {
            return false;
        }
        if (response == 0xFFU) {
            return true;
        }
        if ((i & 0xFFFFU) == 0U) {
            stm32_watchdog_pat();
        }
    }
    return false;
}

static uint8_t spi_command_crc(uint8_t command)
{
    if (command == MMCSD_CMD_GO_IDLE_STATE) {
        return 0x95U;
    }
    if (command == MMCSD_CMD_SEND_IF_COND) {
        return 0x87U;
    }
    return 0x01U;
}

static bool spi_command_selected(uint8_t command, uint32_t argument,
                                 uint8_t &r1, uint8_t *extra = nullptr,
                                 uint8_t extra_length = 0)
{
    const uint8_t header[6] = {
        uint8_t(0x40U | command),
        uint8_t(argument >> 24U),
        uint8_t(argument >> 16U),
        uint8_t(argument >> 8U),
        uint8_t(argument),
        spi_command_crc(command),
    };
    for (uint8_t byte : header) {
        if (!spi_send(byte)) {
            return false;
        }
    }

    r1 = 0xFFU;
    for (uint8_t i = 0; i < CRASHDUMP_SPI_RESPONSE_BYTES; i++) {
        if (!spi_exchange(0xFFU, r1)) {
            return false;
        }
        if ((r1 & 0x80U) == 0U) {
            break;
        }
    }
    if ((r1 & 0x80U) != 0U) {
        return false;
    }
    for (uint8_t i = 0; i < extra_length; i++) {
        if (!spi_exchange(0xFFU, extra[i])) {
            return false;
        }
    }
    return true;
}

static bool spi_command(uint8_t command, uint32_t argument, uint8_t &r1,
                        uint8_t *extra = nullptr, uint8_t extra_length = 0)
{
    spi_select();
    const bool idle = command == MMCSD_CMD_GO_IDLE_STATE || spi_wait_idle();
    const bool success = idle && spi_command_selected(command, argument, r1,
                                                       extra, extra_length);
    spi_unselect();
    return success;
}

static void spi_configure(uint32_t config1, uint32_t config2)
{
    SPI_TypeDef *const spi = sd_spip->spi;
#if defined(STM32H7)
    spi->CR1 &= ~SPI_CR1_SPE;
    spi->CR1 = SPI_CR1_MASRX;
    spi->CR2 = 0U;
    spi->CFG1 = config1 & ~(SPI_CFG1_FTHLV_Msk | SPI_CFG1_RXDMAEN |
                           SPI_CFG1_TXDMAEN);
    spi->CFG2 = (config2 | SPI_CFG2_MASTER | SPI_CFG2_SSOE) &
                ~SPI_CFG2_COMM_Msk;
    spi->IER = 0U;
    spi->IFCR = 0xFFFFFFFFU;
    spi->CR1 |= SPI_CR1_SPE;
#else
    spi->CR2 = 0U;
    spi->CR1 &= ~SPI_CR1_SPE;
    (void)spi->DR;
    (void)spi->SR;
    spi->CR1 = config1 | SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;
#if defined(SPI_CR2_FRXTH)
    config2 |= SPI_CR2_FRXTH;
#endif
    spi->CR2 = config2 & ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
    spi->CR1 |= SPI_CR1_SPE;
#endif
}

static void spi_disable_dma()
{
#if defined(STM32H7)
#if defined(STM32_SPI_DMA_REQUIRED) && defined(STM32_SPI_BDMA_REQUIRED)
    if (sd_spip->is_bdma) {
        if (sd_spip->tx.bdma != nullptr) {
            nvicDisableVector(sd_spip->tx.bdma->vector);
            bdmaStreamDisable(sd_spip->tx.bdma);
        }
        if (sd_spip->rx.bdma != nullptr) {
            nvicDisableVector(sd_spip->rx.bdma->vector);
            bdmaStreamDisable(sd_spip->rx.bdma);
        }
    } else {
        if (sd_spip->tx.dma != nullptr) {
            nvicDisableVector(sd_spip->tx.dma->vector);
            dmaStreamDisable(sd_spip->tx.dma);
        }
        if (sd_spip->rx.dma != nullptr) {
            nvicDisableVector(sd_spip->rx.dma->vector);
            dmaStreamDisable(sd_spip->rx.dma);
        }
    }
#elif defined(STM32_SPI_BDMA_REQUIRED)
    if (sd_spip->tx.bdma != nullptr) {
        nvicDisableVector(sd_spip->tx.bdma->vector);
        bdmaStreamDisable(sd_spip->tx.bdma);
    }
    if (sd_spip->rx.bdma != nullptr) {
        nvicDisableVector(sd_spip->rx.bdma->vector);
        bdmaStreamDisable(sd_spip->rx.bdma);
    }
#elif defined(STM32_SPI_DMA_REQUIRED)
    if (sd_spip->tx.dma != nullptr) {
        nvicDisableVector(sd_spip->tx.dma->vector);
        dmaStreamDisable(sd_spip->tx.dma);
    }
    if (sd_spip->rx.dma != nullptr) {
        nvicDisableVector(sd_spip->rx.dma->vector);
        dmaStreamDisable(sd_spip->rx.dma);
    }
#endif
#else
    if (sd_spip->dmatx != nullptr) {
        nvicDisableVector(sd_spip->dmatx->vector);
        dmaStreamDisable(sd_spip->dmatx);
    }
    if (sd_spip->dmarx != nullptr) {
        nvicDisableVector(sd_spip->dmarx->vector);
        dmaStreamDisable(sd_spip->dmarx);
    }
#endif
}

static void spi_prepare_peripheral()
{
    spi_disable_dma();
    bouncebuffer_abort(sd_spi_bouncebuffer);
    sd_spi_device->crashdump_deassert_all_cs();
    spi_configure(sd_spi_low_config1, sd_spi_low_config2);
}

static bool spi_reconnect_card()
{
    palSetLine(sd_spi_cs_line);
    if (!spi_clock_bytes(16U)) {
        return false;
    }

    uint8_t r1 = 0xFFU;
    bool idle = false;
    for (uint8_t retry = 0; retry < 10U; retry++) {
        if (spi_command(MMCSD_CMD_GO_IDLE_STATE, 0U, r1) && r1 == 0x01U) {
            idle = true;
            break;
        }
    }
    if (!idle) {
        return false;
    }

    uint8_t response[4];
    if (!spi_command(MMCSD_CMD_SEND_IF_COND, MMCSD_CMD8_PATTERN, r1,
                     response, sizeof(response)) ||
        (r1 != 0x01U && r1 != 0x05U)) {
        return false;
    }

    sd_spi_block_addresses = false;
    if (r1 != 0x05U) {
        bool ready = false;
        for (uint32_t retry = 0; retry < 10000U; retry++) {
            if (spi_command(MMCSD_CMD_APP_CMD, 0U, r1) && r1 <= 0x01U &&
                spi_command(MMCSD_CMD_APP_OP_COND, 0x400001AAU, r1) && r1 == 0x00U) {
                ready = true;
                break;
            }
            if ((retry & 0xFFU) == 0U) {
                stm32_watchdog_pat();
            }
        }
        if (!ready ||
            !spi_command(MMCSD_CMD_READ_OCR, 0U, r1, response, sizeof(response)) ||
            r1 != 0x00U) {
            return false;
        }
        sd_spi_block_addresses = (response[0] & 0x40U) != 0U;
    } else {
        bool ready = false;
        for (uint32_t retry = 0; retry < 10000U; retry++) {
            if (spi_command(MMCSD_CMD_INIT, 0U, r1) && r1 == 0x00U) {
                ready = true;
                break;
            }
            if ((retry & 0xFFU) == 0U) {
                stm32_watchdog_pat();
            }
        }
        if (!ready) {
            return false;
        }
    }

    spi_configure(sd_spi_high_config1, sd_spi_high_config2);
    return spi_command(MMCSD_CMD_SET_BLOCKLEN, MMCSD_BLOCK_SIZE, r1) && r1 == 0x00U;
}

static uint32_t spi_card_address(uint32_t sector)
{
    return sd_spi_block_addresses ? sector : sector * MMCSD_BLOCK_SIZE;
}

static bool wait_for_transfer_state()
{
    uint8_t r1;
    uint8_t r2;
    return spi_command(MMCSD_CMD_SEND_STATUS, 0U, r1, &r2, 1U) &&
           r1 == 0x00U && r2 == 0x00U;
}

static bool abort_transfer()
{
    spi_prepare_peripheral();
    return spi_reconnect_card();
}

static bool read_blocks(uint32_t sector, uint32_t blocks)
{
    uint8_t *buffer = sd_dma_buf;
    while (blocks-- > 0U) {
        spi_select();
        uint8_t r1;
        bool success = spi_wait_idle() &&
                       spi_command_selected(MMCSD_CMD_READ_SINGLE_BLOCK,
                                            spi_card_address(sector), r1) &&
                       r1 == 0x00U;
        uint8_t token = 0xFFU;
        for (uint32_t i = 0; success && i < CRASHDUMP_SPI_BUSY_BYTES; i++) {
            success = spi_exchange(0xFFU, token);
            if (!success || token != 0xFFU) {
                break;
            }
        }
        success = success && token == 0xFEU;
        for (uint32_t i = 0; success && i < MMCSD_BLOCK_SIZE; i++) {
            success = spi_exchange(0xFFU, buffer[i]);
        }
        success = success && spi_clock_bytes(2U);
        spi_unselect();
        if (!success) {
            return false;
        }
        stm32_watchdog_pat();
        buffer += MMCSD_BLOCK_SIZE;
        sector++;
    }
    return true;
}

static bool write_blocks(uint32_t sector, uint32_t blocks)
{
    const bool multiple = blocks > 1U;
    spi_select();
    uint8_t r1;
    bool success = spi_wait_idle() &&
                   spi_command_selected(multiple ? MMCSD_CMD_WRITE_MULTIPLE_BLOCK :
                                                  MMCSD_CMD_WRITE_BLOCK,
                                        spi_card_address(sector), r1) &&
                   r1 == 0x00U;
    const uint8_t *buffer = sd_dma_buf;
    for (uint32_t block = 0; success && block < blocks; block++) {
        success = spi_send(multiple ? 0xFCU : 0xFEU);
        for (uint32_t i = 0; success && i < MMCSD_BLOCK_SIZE; i++) {
            success = spi_send(buffer[i]);
        }
        success = success && spi_clock_bytes(2U);
        uint8_t response = 0xFFU;
        success = success && spi_exchange(0xFFU, response) &&
                  (response & 0x1FU) == 0x05U && spi_wait_idle();
        stm32_watchdog_pat();
        buffer += MMCSD_BLOCK_SIZE;
    }
    if (multiple) {
        success = spi_send(0xFDU) && spi_wait_idle() && success;
    }
    spi_unselect();
    return success;
}

#else // CRASHDUMP_SD_SPI

#define CRASHDUMP_SDC_COMMAND_POLL_LIMIT 10000000U
#define CRASHDUMP_SDC_TRANSFER_STATE_POLLS 1000000U
#define CRASHDUMP_SDC_DATA_POLL_LIMIT 100000000U
#define CRASHDUMP_SDC_WATCHDOG_POLL_MASK 0xFFFFU

/*
  ChibiOS' command helpers have no software bound around their peripheral
  status loops. Keep the crash-time path bounded in case the controller is
  wedged rather than relying solely on its command timeout flag.
 */
static bool send_command_short_crc(uint8_t command, uint32_t argument,
                                   uint32_t &response)
{
    uint32_t status = 0;
#if CRASHDUMP_SD_SDMMCV2 || CRASHDUMP_SD_SDMMCV1
    const uint32_t success_flag = SDMMC_STA_CMDREND;
    const uint32_t error_flags = SDMMC_STA_CTIMEOUT | SDMMC_STA_CCRCFAIL;
    const uint32_t completion_flags = success_flag | error_flags;
    sd_sdcp->sdmmc->ARG = argument;
    sd_sdcp->sdmmc->CMD = uint32_t(command) | SDMMC_CMD_WAITRESP_0 |
                          SDMMC_CMD_CPSMEN;
#elif CRASHDUMP_SD_SDIOV1
    const uint32_t success_flag = SDIO_STA_CMDREND;
    const uint32_t error_flags = SDIO_STA_CTIMEOUT | SDIO_STA_CCRCFAIL;
    const uint32_t completion_flags = success_flag | error_flags;
    sd_sdcp->sdio->ARG = argument;
    sd_sdcp->sdio->CMD = uint32_t(command) | SDIO_CMD_WAITRESP_0 |
                         SDIO_CMD_CPSMEN;
#endif

    for (uint32_t i = 0; i < CRASHDUMP_SDC_COMMAND_POLL_LIMIT; i++) {
#if CRASHDUMP_SD_SDMMCV2 || CRASHDUMP_SD_SDMMCV1
        status = sd_sdcp->sdmmc->STA;
#elif CRASHDUMP_SD_SDIOV1
        status = sd_sdcp->sdio->STA;
#endif
        if ((status & completion_flags) != 0U) {
            break;
        }
        if ((i & CRASHDUMP_SDC_WATCHDOG_POLL_MASK) == 0U) {
            stm32_watchdog_pat();
        }
    }

#if CRASHDUMP_SD_SDMMCV2 || CRASHDUMP_SD_SDMMCV1
    if ((status & completion_flags) == 0U) {
        sd_sdcp->sdmmc->CMD = 0U;
    }
    sd_sdcp->sdmmc->ICR = status & completion_flags;
    response = sd_sdcp->sdmmc->RESP1;
#elif CRASHDUMP_SD_SDIOV1
    if ((status & completion_flags) == 0U) {
        sd_sdcp->sdio->CMD = 0U;
    }
    sd_sdcp->sdio->ICR = status & completion_flags;
    response = sd_sdcp->sdio->RESP1;
#endif
    return (status & success_flag) != 0U && (status & error_flags) == 0U;
}

/*
  Poll for the card transfer state without sleeping or taking an RTOS lock.
  ChibiOS' command primitive is itself polling-only.
 */
static bool wait_for_transfer_state()
{
    bool ignore_first_error = true;
    for (uint32_t i = 0; i < CRASHDUMP_SDC_TRANSFER_STATE_POLLS; i++) {
        uint32_t response;
        const bool command_ok = send_command_short_crc(
                                    MMCSD_CMD_SEND_STATUS, sd_sdcp->rca, response);
        if (!command_ok || MMCSD_R1_ERROR(response)) {
            if (ignore_first_error) {
                ignore_first_error = false;
                continue;
            }
            return false;
        }
        ignore_first_error = false;
        if (MMCSD_R1_STS(response) == MMCSD_STS_TRAN) {
            return true;
        }
        if ((i & 0xFFU) == 0U) {
            stm32_watchdog_pat();
        }
    }
    return false;
}

static uint32_t data_timeout_ticks(uint32_t timeout_ms)
{
#if CRASHDUMP_SD_SDMMCV2
    const uint32_t divider = (sd_sdcp->sdmmc->CLKCR & SDMMC_CLKCR_CLKDIV_Msk) + 1U;
    return ((sd_sdcp->clkfreq / (divider * 2U)) / 1000U) * timeout_ms;
#elif CRASHDUMP_SD_SDMMCV1
    const uint32_t clkcr = sd_sdcp->sdmmc->CLKCR;
    const uint32_t divider = (clkcr & SDMMC_CLKCR_BYPASS) != 0U ?
                             1U : (clkcr & SDMMC_CLKCR_CLKDIV_Msk) + 2U;
    return ((sd_sdcp->clkfreq / (divider * 2U)) / 1000U) * timeout_ms;
#elif CRASHDUMP_SD_SDIOV1
    const uint32_t clkcr = sd_sdcp->sdio->CLKCR;
    const uint32_t divider = (clkcr & SDIO_CLKCR_BYPASS) != 0U ?
                             1U : (clkcr & SDIO_CLKCR_CLKDIV_Msk) + 2U;
    return ((48000000U / (divider * 2U)) / 1000U) * timeout_ms;
#endif
}

static void stop_data_path()
{
#if CRASHDUMP_SD_SDMMCV2
    sd_sdcp->sdmmc->IDMACTRL = 0U;
    sd_sdcp->sdmmc->MASK = 0U;
    sd_sdcp->sdmmc->DCTRL = 0U;
#elif CRASHDUMP_SD_SDMMCV1
    sd_sdcp->sdmmc->MASK = 0U;
    if (sd_sdcp->dma != nullptr) {
        dmaStreamDisable(sd_sdcp->dma);
    }
    sd_sdcp->sdmmc->DCTRL = 0U;
#elif CRASHDUMP_SD_SDIOV1
    sd_sdcp->sdio->MASK = 0U;
    if (sd_sdcp->dma != nullptr) {
        dmaStreamDisable(sd_sdcp->dma);
    }
    sd_sdcp->sdio->DCTRL = 0U;
#endif
}

static void clear_data_flags()
{
#if CRASHDUMP_SD_SDMMCV2
    sd_sdcp->sdmmc->ICR = SDMMC_ICR_ALL_FLAGS;
#elif CRASHDUMP_SD_SDMMCV1
    sd_sdcp->sdmmc->ICR = SDMMC_ICR_ALL_FLAGS;
#elif CRASHDUMP_SD_SDIOV1
    sd_sdcp->sdio->ICR = CRASHDUMP_SDIO_ICR_ALL_FLAGS;
#endif
}

static bool stop_multiblock_transfer()
{
    uint32_t response;
    return send_command_short_crc(MMCSD_CMD_STOP_TRANSMISSION, 0, response);
}

static bool wait_for_data_end(uint32_t blocks)
{
    uint32_t status = 0;
#if CRASHDUMP_SD_SDMMCV2
    for (uint32_t i = 0; i < CRASHDUMP_SDC_DATA_POLL_LIMIT; i++) {
        status = sd_sdcp->sdmmc->STA;
        if ((status & (SDMMC_STA_DATAEND | SDMMC_DATA_ERROR_FLAGS)) != 0) {
            break;
        }
        if ((i & CRASHDUMP_SDC_WATCHDOG_POLL_MASK) == 0U) {
            stm32_watchdog_pat();
        }
    }

    stop_data_path();
    clear_data_flags();

    const bool success = (status & SDMMC_STA_DATAEND) != 0 &&
                         (status & SDMMC_DATA_ERROR_FLAGS) == 0;
#elif CRASHDUMP_SD_SDMMCV1
    bool dma_complete = false;
    for (uint32_t i = 0; i < CRASHDUMP_SDC_DATA_POLL_LIMIT; i++) {
        status = sd_sdcp->sdmmc->STA;
        if ((status & SDMMC_DATA_ERROR_FLAGS) != 0U) {
            break;
        }
        // F7 uses DMAv2 peripheral flow control; match dmaWaitCompletion().
        if ((status & SDMMC_STA_DATAEND) != 0U &&
            (sd_sdcp->dma->stream->CR & STM32_DMA_CR_EN) == 0U) {
            dma_complete = true;
            break;
        }
        if ((i & CRASHDUMP_SDC_WATCHDOG_POLL_MASK) == 0U) {
            stm32_watchdog_pat();
        }
    }

    stop_data_path();
    clear_data_flags();

    const bool success = dma_complete &&
                         (status & SDMMC_STA_DATAEND) != 0U &&
                         (status & SDMMC_DATA_ERROR_FLAGS) == 0U;
#elif CRASHDUMP_SD_SDIOV1
    bool dma_complete = false;
    for (uint32_t i = 0; i < CRASHDUMP_SDC_DATA_POLL_LIMIT; i++) {
        status = sd_sdcp->sdio->STA;
        if ((status & SDIO_DATA_ERROR_FLAGS) != 0U) {
            break;
        }
        if ((status & SDIO_STA_DATAEND) != 0U &&
            dmaStreamGetTransactionSize(sd_sdcp->dma) == 0U) {
            dma_complete = true;
            break;
        }
        if ((i & CRASHDUMP_SDC_WATCHDOG_POLL_MASK) == 0U) {
            stm32_watchdog_pat();
        }
    }

    stop_data_path();
    clear_data_flags();

    const bool success = dma_complete &&
                         (status & SDIO_STA_DATAEND) != 0U &&
                         (status & SDIO_DATA_ERROR_FLAGS) == 0U;
#endif
    const bool stopped = blocks <= 1U || stop_multiblock_transfer();
    return success && stopped;
}

static bool abort_transfer()
{
#if CRASHDUMP_SD_SDMMCV2 || CRASHDUMP_SD_SDMMCV1
#if defined(STM32_SDC_USE_SDMMC1) && STM32_SDC_USE_SDMMC1 == TRUE
    if (sd_sdcp == &SDCD1) {
        nvicDisableVector(STM32_SDMMC1_NUMBER);
    }
#endif
#if defined(STM32_SDC_USE_SDMMC2) && STM32_SDC_USE_SDMMC2 == TRUE
    if (sd_sdcp == &SDCD2) {
        nvicDisableVector(STM32_SDMMC2_NUMBER);
    }
#endif
#if CRASHDUMP_SD_SDMMCV1
    if (sd_sdcp->dma != nullptr) {
        nvicDisableVector(sd_sdcp->dma->vector);
    }
#endif
#elif CRASHDUMP_SD_SDIOV1
    nvicDisableVector(STM32_SDIO_NUMBER);
    if (sd_sdcp->dma != nullptr) {
        nvicDisableVector(sd_sdcp->dma->vector);
    }
#endif

    stop_data_path();
    clear_data_flags();

    for (volatile uint32_t i = 0; i < 1000U; i++) {
    }
    for (uint8_t i = 0; i < 3U; i++) {
        (void)stop_multiblock_transfer();
        clear_data_flags();
    }
    stop_data_path();
    clear_data_flags();
    bouncebuffer_abort(sd_sdcp->bouncebuffer);
    return true;
}

static bool prepare_data_transfer(uint32_t start_sector, uint32_t blocks, bool write)
{
    uint32_t card_address = start_sector;
    if ((sd_sdcp->cardmode & SDC_MODE_HIGH_CAPACITY) == 0) {
        card_address *= MMCSD_BLOCK_SIZE;
    }

#if CRASHDUMP_SD_SDMMCV2
    const uint32_t timeout_ms = write ?
                                STM32_SDC_SDMMC_WRITE_TIMEOUT : STM32_SDC_SDMMC_READ_TIMEOUT;
    sd_sdcp->sdmmc->DTIMER = data_timeout_ticks(timeout_ms);
#elif CRASHDUMP_SD_SDMMCV1
    const uint32_t timeout_ms = write ?
                                STM32_SDC_SDMMC_WRITE_TIMEOUT : STM32_SDC_SDMMC_READ_TIMEOUT;
    sd_sdcp->sdmmc->DTIMER = data_timeout_ticks(timeout_ms);
#elif CRASHDUMP_SD_SDIOV1
    const uint32_t timeout_ms = write ? STM32_SDC_WRITE_TIMEOUT_MS : STM32_SDC_READ_TIMEOUT_MS;
    sd_sdcp->sdio->DTIMER = data_timeout_ticks(timeout_ms);
#endif
    if (!wait_for_transfer_state()) {
        return false;
    }
#if CRASHDUMP_SD_SDMMCV2
    sd_sdcp->sdmmc->IDMABASE0 = reinterpret_cast<uint32_t>(sd_dma_buf);
    sd_sdcp->sdmmc->IDMACTRL = SDMMC_IDMA_IDMAEN;
    sd_sdcp->sdmmc->ICR = SDMMC_ICR_ALL_FLAGS;
    sd_sdcp->sdmmc->MASK = 0U;
    sd_sdcp->sdmmc->DLEN = blocks * MMCSD_BLOCK_SIZE;
#elif CRASHDUMP_SD_SDMMCV1
    if (sd_sdcp->dma == nullptr) {
        return false;
    }
    dmaStreamSetMemory0(sd_sdcp->dma, sd_dma_buf);
    dmaStreamSetTransactionSize(sd_sdcp->dma,
                                blocks * MMCSD_BLOCK_SIZE / sizeof(uint32_t));
    dmaStreamSetMode(sd_sdcp->dma, sd_sdcp->dmamode |
                    (write ? STM32_DMA_CR_DIR_M2P : STM32_DMA_CR_DIR_P2M));
    dmaStreamEnable(sd_sdcp->dma);

    sd_sdcp->sdmmc->ICR = SDMMC_ICR_ALL_FLAGS;
    sd_sdcp->sdmmc->MASK = 0U;
    sd_sdcp->sdmmc->DLEN = blocks * MMCSD_BLOCK_SIZE;
#elif CRASHDUMP_SD_SDIOV1
    if (sd_sdcp->dma == nullptr) {
        return false;
    }
    dmaStreamSetMemory0(sd_sdcp->dma, sd_dma_buf);
    dmaStreamSetTransactionSize(sd_sdcp->dma,
                                blocks * MMCSD_BLOCK_SIZE / sizeof(uint32_t));
    dmaStreamSetMode(sd_sdcp->dma, sd_sdcp->dmamode |
                    (write ? STM32_DMA_CR_DIR_M2P : STM32_DMA_CR_DIR_P2M));
    dmaStreamEnable(sd_sdcp->dma);

    sd_sdcp->sdio->ICR = CRASHDUMP_SDIO_ICR_ALL_FLAGS;
    sd_sdcp->sdio->MASK = 0U;
    sd_sdcp->sdio->DLEN = blocks * MMCSD_BLOCK_SIZE;
#endif

    const uint8_t command = write ?
                            (blocks > 1U ? MMCSD_CMD_WRITE_MULTIPLE_BLOCK : MMCSD_CMD_WRITE_BLOCK) :
                            (blocks > 1U ? MMCSD_CMD_READ_MULTIPLE_BLOCK : MMCSD_CMD_READ_SINGLE_BLOCK);
    uint32_t response;
    if (!send_command_short_crc(command, card_address, response) ||
        MMCSD_R1_ERROR(response)) {
        stop_data_path();
        return false;
    }
#if CRASHDUMP_SD_SDMMCV2
    sd_sdcp->sdmmc->DCTRL = (write ? 0U : SDMMC_DCTRL_DTDIR) |
                            SDMMC_DCTRL_FIFORST |
                            SDMMC_DCTRL_DBLOCKSIZE_3 |
                            SDMMC_DCTRL_DBLOCKSIZE_0 |
                            SDMMC_DCTRL_DTEN;
#elif CRASHDUMP_SD_SDMMCV1
    sd_sdcp->sdmmc->DCTRL = (write ? 0U : SDMMC_DCTRL_DTDIR) |
                            SDMMC_DCTRL_DBLOCKSIZE_3 |
                            SDMMC_DCTRL_DBLOCKSIZE_0 |
                            SDMMC_DCTRL_DMAEN |
                            SDMMC_DCTRL_DTEN;
#elif CRASHDUMP_SD_SDIOV1
    sd_sdcp->sdio->DCTRL = (write ? 0U : SDIO_DCTRL_DTDIR) |
                           SDIO_DCTRL_DBLOCKSIZE_3 |
                           SDIO_DCTRL_DBLOCKSIZE_0 |
                           SDIO_DCTRL_DMAEN |
                           SDIO_DCTRL_DTEN;
#endif
    return true;
}

static bool read_blocks(uint32_t sector, uint32_t blocks)
{
    stm32_cacheBufferInvalidate(sd_dma_buf, blocks * MMCSD_BLOCK_SIZE);
    if (!prepare_data_transfer(sector, blocks, false) || !wait_for_data_end(blocks)) {
        return false;
    }
    stm32_cacheBufferInvalidate(sd_dma_buf, blocks * MMCSD_BLOCK_SIZE);
    return true;
}

static bool write_blocks(uint32_t sector, uint32_t blocks)
{
    stm32_cacheBufferFlush(sd_dma_buf, blocks * MMCSD_BLOCK_SIZE);
    return prepare_data_transfer(sector, blocks, true) && wait_for_data_end(blocks);
}

#endif // CRASHDUMP_SD_SPI

static uint32_t accumulator_capacity()
{
    uint32_t sector;
    uint32_t contiguous_sectors;
    if (!sector_mapping(sd_write_offset / MMCSD_BLOCK_SIZE,
                        sector, contiguous_sectors)) {
        return 0;
    }
    if (sd_write_offset >= crashdump_sd_max_size()) {
        return 0;
    }
    return min_u32(min_u32(sd_dma_buf_size, contiguous_sectors * MMCSD_BLOCK_SIZE),
                   crashdump_sd_max_size() - sd_write_offset);
}

static bool flush_accumulator()
{
    if (accumulator_offset == 0 ||
        (accumulator_offset % MMCSD_BLOCK_SIZE) != 0) {
        return false;
    }

    uint32_t sector;
    uint32_t contiguous_sectors;
    if (!sector_mapping(sd_write_offset / MMCSD_BLOCK_SIZE,
                        sector, contiguous_sectors)) {
        return false;
    }

    const uint32_t blocks = accumulator_offset / MMCSD_BLOCK_SIZE;
    if (blocks > contiguous_sectors) {
        return false;
    }

    bool success = false;
    for (uint8_t retry = 0; retry < 3U; retry++) {
        if (write_blocks(sector, blocks)) {
            success = true;
            break;
        }
    }
    if (!success) {
        return false;
    }

    sd_write_offset += accumulator_offset;
    accumulator_offset = 0;
    stm32_watchdog_pat();
    return true;
}

bool crashdump_sd_init()
{
    sd_is_ready = false;
    sd_dump_size = 0;
    delete[] sd_extents;
    sd_extents = nullptr;
    sd_extent_count = 0;
    sd_total_sectors = 0;

#if CRASHDUMP_SD_SPI
    sd_mmcp = &MMCD1;
    AP_HAL::SPIDevice *const hal_device = sdcard_get_spi_device();
    if (hal_device == nullptr) {
        return init_failed(CrashDumpDiagnostic::NO_SD_SPI_DEVICE);
    }
    sd_spi_device = static_cast<ChibiOS::SPIDevice *>(hal_device);
    sd_spip = sd_spi_device->get_driver();
    if (sd_spip == nullptr || sd_spip->config == nullptr ||
        sd_mmcp->config == nullptr || sd_mmcp->config->lscfg == nullptr ||
        sd_mmcp->config->hscfg == nullptr) {
        return init_failed(CrashDumpDiagnostic::SD_SPI_NOT_RUNNING);
    }
    sd_spi_cs_line = sd_spi_device->get_chip_select_line();
    sd_spi_device->get_crashdump_config(false, sd_spi_low_config1,
                                        sd_spi_low_config2);
    sd_spi_device->get_crashdump_config(true, sd_spi_high_config1,
                                        sd_spi_high_config2);
    sd_spi_block_addresses = sd_mmcp->block_addresses;

    for (uint32_t size = AP_FATFS_MAX_IO_SIZE;
         size >= MMCSD_BLOCK_SIZE;
         size /= 2U) {
        sd_spi_bouncebuffer = sd_spi_device->prepare_crashdump_buffer(size);
        if (sd_spi_bouncebuffer != nullptr &&
            sd_spi_bouncebuffer->dma_buf != nullptr) {
            break;
        }
    }
    if (sd_spi_bouncebuffer == nullptr ||
        sd_spi_bouncebuffer->dma_buf == nullptr) {
        return init_failed(CrashDumpDiagnostic::NO_SD_SPI_BOUNCEBUFFER);
    }
    sd_dma_buf = sd_spi_bouncebuffer->dma_buf;
    sd_dma_buf_size = sd_spi_bouncebuffer->size & ~(MMCSD_BLOCK_SIZE - 1U);
#else
#if defined(STM32_SDC_USE_SDMMC2) && STM32_SDC_USE_SDMMC2 == TRUE
    sd_sdcp = &SDCD2;
#else
    sd_sdcp = &SDCD1;
#endif

    if (sd_sdcp->bouncebuffer == nullptr ||
        sd_sdcp->bouncebuffer->dma_buf == nullptr) {
        return init_failed(CrashDumpDiagnostic::NO_SD_BOUNCEBUFFER);
    }
    sd_dma_buf = sd_sdcp->bouncebuffer->dma_buf;
    sd_dma_buf_size = sd_sdcp->bouncebuffer->size & ~(MMCSD_BLOCK_SIZE - 1U);
#endif
    if (sd_dma_buf_size < MMCSD_BLOCK_SIZE) {
        return init_failed(CrashDumpDiagnostic::SD_BOUNCEBUFFER_TOO_SMALL,
                           sd_dma_buf_size);
    }

    const uint32_t target_size = crashdump_sd_file_size();
    (void)f_mkdir("APM");
    bool reset_reserved;
    if (!publish_crashdump(reset_reserved)) {
        return init_failed(CrashDumpDiagnostic::PUBLISH_RESERVED);
    }

    const bool armed = hal.util->get_soft_armed();
    bool new_reserved = false;
    FIL fp;
    FRESULT result = f_open(&fp, crashdump_reserved_path,
                            FA_OPEN_EXISTING | FA_READ | FA_WRITE);
    if (result == FR_NO_FILE) {
        if (armed) {
            return init_failed(CrashDumpDiagnostic::RESERVE_CREATE_ARMED);
        }
        result = f_open(&fp, crashdump_reserved_path,
                        FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
        new_reserved = result == FR_OK;
    }
    if (result != FR_OK) {
        return init_failed(CrashDumpDiagnostic::OPEN_RESERVED, result);
    }

    if (reset_reserved || new_reserved || f_size(&fp) != target_size) {
        if (armed) {
            f_close(&fp);
            return init_failed(CrashDumpDiagnostic::RESERVE_CREATE_ARMED);
        }
        if (!new_reserved) {
            f_close(&fp);
            result = f_open(&fp, crashdump_reserved_path,
                            FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
            if (result != FR_OK) {
                return init_failed(CrashDumpDiagnostic::RECREATE_RESERVED, result);
            }
        }

        memset(sd_dma_buf, 0xFF, sd_dma_buf_size);
        for (uint32_t offset = 0; offset < target_size; offset += sd_dma_buf_size) {
            const UINT chunk = min_u32(sd_dma_buf_size, target_size - offset);
            UINT bytes_written;
            result = f_write(&fp, sd_dma_buf, chunk, &bytes_written);
            if (result != FR_OK || bytes_written != chunk) {
                f_close(&fp);
                return init_failed(CrashDumpDiagnostic::INITIALISE_RESERVED, result);
            }
            stm32_watchdog_pat();
        }
        stm32_watchdog_pat();
        result = f_sync(&fp);
        if (result != FR_OK) {
            f_close(&fp);
            return init_failed(CrashDumpDiagnostic::SYNC_RESERVED, result);
        }
        stm32_watchdog_pat();
    }

    const uint32_t file_size = f_size(&fp);
    f_close(&fp);
    if (file_size < 2U * MMCSD_BLOCK_SIZE ||
        (file_size % MMCSD_BLOCK_SIZE) != 0U) {
        return init_failed(CrashDumpDiagnostic::INVALID_RESERVED_SIZE, file_size);
    }

    result = f_open(&fp, crashdump_reserved_path, FA_READ);
    if (result != FR_OK) {
        return init_failed(CrashDumpDiagnostic::REOPEN_RESERVED, result);
    }
    const uint8_t filesystem_type = fp.obj.fs->fs_type;
    const bool extent_list_ok = build_extent_list(fp, file_size / MMCSD_BLOCK_SIZE);
    f_close(&fp);
    if (!extent_list_ok) {
        return false;
    }

    calculate_firmware_identity();
    if (get_dump_state(crashdump_published_path, sd_dump_size) !=
        CrashDumpFileState::COMPLETE) {
        sd_dump_size = 0;
    }
    sd_dump_check_ms = AP_HAL::millis();

    printf("CrashDumpSD: fs %u, %u extents, %u sectors, %u byte buffer, firmware %08x/%u\n",
           unsigned(filesystem_type),
           unsigned(sd_extent_count), unsigned(sd_total_sectors),
           unsigned(sd_dma_buf_size), unsigned(sd_firmware_crc),
           unsigned(sd_firmware_size));
    sd_is_ready = true;
    return true;
}

bool crashdump_sd_ready()
{
    return sd_is_ready;
}

uint32_t crashdump_sd_max_size()
{
    if (sd_total_sectors < 2U) {
        return 0;
    }
    return (sd_total_sectors - 1U) * MMCSD_BLOCK_SIZE;
}

bool crashdump_sd_start()
{
    if (!sd_is_ready || sd_dma_buf == nullptr
#if CRASHDUMP_SD_SPI
        || sd_mmcp == nullptr || sd_spi_device == nullptr || sd_spip == nullptr
#else
        || sd_sdcp == nullptr
#endif
       ) {
        return false;
    }

    if (!abort_transfer()) {
        return false;
    }
    stm32_watchdog_pat();

    uint32_t first_sector;
    uint32_t available;
    if (!sector_mapping(0, first_sector, available) ||
        !read_blocks(first_sector, 1U)) {
        return false;
    }
    stm32_watchdog_pat();

    if (sd_dma_buf[0] == 0x63 && sd_dma_buf[1] == 0x43) {
        return false;
    }

    sd_write_offset = 0;
    accumulator_offset = 0;
    sd_write_failed = false;
    return true;
}

bool crashdump_sd_write(const uint8_t *data, uint32_t length)
{
    if (sd_write_failed) {
        return false;
    }

    while (length > 0) {
        const uint32_t capacity = accumulator_capacity();
        if (capacity == 0) {
            sd_write_failed = true;
            return false;
        }

        const uint32_t space = capacity - accumulator_offset;
        const uint32_t chunk = min_u32(length, space);
        memmove(&sd_dma_buf[accumulator_offset], data, chunk);
        accumulator_offset += chunk;
        data += chunk;
        length -= chunk;

        if (accumulator_offset == capacity && !flush_accumulator()) {
            sd_write_failed = true;
            return false;
        }
    }
    return true;
}

bool crashdump_sd_end(uint32_t dump_size)
{
    if (sd_write_failed || dump_size > crashdump_sd_max_size()) {
        return false;
    }

    if (accumulator_offset > 0) {
        const uint32_t padded_size = (accumulator_offset + MMCSD_BLOCK_SIZE - 1U) &
                                     ~(MMCSD_BLOCK_SIZE - 1U);
        memset(&sd_dma_buf[accumulator_offset], 0xFF, padded_size - accumulator_offset);
        accumulator_offset = padded_size;
        if (!flush_accumulator()) {
            return false;
        }
    }

    uint32_t last_sector;
    uint32_t available;
    if (!sector_mapping(sd_total_sectors - 1U, last_sector, available)) {
        return false;
    }
    memset(sd_dma_buf, 0xFF, MMCSD_BLOCK_SIZE);
    CrashDumpTrailer trailer {};
    memcpy(trailer.magic, crashdump_trailer_magic, sizeof(trailer.magic));
    trailer.version = CRASHDUMP_TRAILER_VERSION;
    trailer.size = sizeof(trailer);
    trailer.git_hash = sd_firmware_git_hash;
    trailer.firmware_crc = sd_firmware_crc;
    trailer.firmware_size = sd_firmware_size;
    trailer.dump_size = dump_size;
    trailer.trailer_crc = crc_crc32(0, reinterpret_cast<const uint8_t *>(&trailer),
                                    sizeof(trailer));
    memcpy(&sd_dma_buf[MMCSD_BLOCK_SIZE - sizeof(trailer)], &trailer, sizeof(trailer));
    return write_blocks(last_sector, 1U) && wait_for_transfer_state();
}

uint32_t crashdump_sd_dump_size()
{
    if (!sd_is_ready || sd_dump_size == 0) {
        return 0;
    }

    // MAVFTP can remove the published file without going through this module.
    // Refresh at a low rate so the arming check clears without a reboot.
    const uint32_t now = AP_HAL::millis();
    if (now - sd_dump_check_ms >= 1000U) {
        sd_dump_check_ms = now;
        struct stat st;
        if (AP::FS().stat(crashdump_published_path, &st) != 0 && errno == ENOENT) {
            sd_dump_size = 0;
        }
    }
    return sd_dump_size;
}

#endif // AP_CRASHDUMP_FATFS_ENABLED && SD card transport
