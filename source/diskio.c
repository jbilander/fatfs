/**
 * diskio.c - FatFs low-level disk I/O layer for RP2350 SPI0 SD card
 * Debug-enabled write tracing
 */

#include "main.h"
#include "ff.h"
#include "diskio.h"
#include "pico/mutex.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include "pico/stdio.h"
#include <string.h>

// -----------------------------------------------------------------------------
// Shared global mutex declared in main.h
extern mutex_t spi_mutex;

// -----------------------------------------------------------------------------
// SPI helper macros
#define SD_CMD0     0x00
#define SD_CMD8     0x08
#define SD_CMD17    0x11
#define SD_CMD24    0x18
#define SD_CMD55    0x37
#define SD_CMD58    0x3A
#define SD_ACMD41   0x29
#define SD_CMD16    0x10
#define SD_TOKEN_START_BLOCK 0xFE

#define SD_SECTOR_SIZE 512
#define SD_DUMMY_BYTE  0xFF

// -----------------------------------------------------------------------------
// Local state
static bool card_initialized = false;

// -----------------------------------------------------------------------------
// SPI helpers
static inline void sd_cs_low(void)  { gpio_put(PIN_SS, 0); }
static inline void sd_cs_high(void) { gpio_put(PIN_SS, 1); }

static uint8_t spi_txrx(uint8_t data) {
    uint8_t rx;
    spi_write_read_blocking(spi0, &data, &rx, 1);
    return rx;
}

static void spi_write_buf(const uint8_t *buf, size_t len) {
    spi_write_blocking(spi0, buf, len);
}

static void spi_read_buf(uint8_t *buf, size_t len) {
    spi_read_blocking(spi0, SD_DUMMY_BYTE, buf, len);
}

static void spi_send_dummy_clocks(uint32_t count) {
    for (uint32_t i = 0; i < count; i++) spi_txrx(SD_DUMMY_BYTE);
}

// -----------------------------------------------------------------------------
// SD command helpers
static uint8_t sd_send_cmd(uint8_t cmd, uint32_t arg, uint8_t crc) {
    uint8_t buf[6];
    buf[0] = 0x40 | cmd;
    buf[1] = (uint8_t)(arg >> 24);
    buf[2] = (uint8_t)(arg >> 16);
    buf[3] = (uint8_t)(arg >> 8);
    buf[4] = (uint8_t)arg;
    buf[5] = crc;

    sd_cs_low();
    spi_write_buf(buf, 6);

    uint8_t resp;
    for (int i = 0; i < 8; i++) {
        resp = spi_txrx(SD_DUMMY_BYTE);
        if ((resp & 0x80) == 0) break;
    }

    return resp;
}

static bool sd_wait_ready(uint32_t timeout_ms) {
    absolute_time_t end = make_timeout_time_ms(timeout_ms);
    uint8_t r;
    do {
        r = spi_txrx(SD_DUMMY_BYTE);
        if (r == 0xFF) return true;
    } while (!time_reached(end));
    return false;
}

static bool sd_send_data_block(const uint8_t *buff, uint8_t token) {
    if (!sd_wait_ready(500)) {
        printf("Debug: SD not ready before write\n");
        return false;
    }

    spi_txrx(token);
    spi_write_buf(buff, SD_SECTOR_SIZE);

    spi_txrx(0xFF);
    spi_txrx(0xFF);

    uint8_t resp = spi_txrx(SD_DUMMY_BYTE);
    if ((resp & 0x1F) != 0x05) {
        printf("Debug: Data reject, resp=0x%02X\n", resp);
        return false;
    }

    // Wait until card is ready again (write complete)
    if (!sd_wait_ready(500)) {
        printf("Debug: Card not ready after write\n");
        return false;
    }

    return true;
}

static bool sd_receive_data_block(uint8_t *buff, uint32_t bytes) {
    absolute_time_t end = make_timeout_time_ms(100);
    uint8_t token;
    do {
        token = spi_txrx(SD_DUMMY_BYTE);
        if (token == SD_TOKEN_START_BLOCK) break;
    } while (!time_reached(end));

    if (token != SD_TOKEN_START_BLOCK) return false;

    spi_read_buf(buff, bytes);
    spi_txrx(SD_DUMMY_BYTE);
    spi_txrx(SD_DUMMY_BYTE);
    return true;
}

// -----------------------------------------------------------------------------
// Disk I/O functions
DSTATUS disk_initialize(BYTE pdrv) {
    (void)pdrv;

    mutex_enter_blocking(&spi_mutex);
    spi_set_baudrate(spi0, SPI_SLOW_FREQUENCY);

    sd_cs_high();
    spi_send_dummy_clocks(10);

    uint8_t resp;
    int tries = 1000;
    do {
        resp = sd_send_cmd(SD_CMD0, 0, 0x95);
    } while (resp != 0x01 && --tries);

    if (resp != 0x01) {
        mutex_exit(&spi_mutex);
        return STA_NOINIT;
    }

    resp = sd_send_cmd(SD_CMD8, 0x1AA, 0x87);
    if ((resp & 0x04) == 0) {
        uint8_t ocr[4];
        spi_read_buf(ocr, 4);
    }

    tries = 2000;
    do {
        sd_send_cmd(SD_CMD55, 0, 0x65);
        resp = sd_send_cmd(SD_ACMD41, 0x40000000, 0x77);
    } while (resp != 0x00 && --tries);

    sd_send_cmd(SD_CMD58, 0, 0);
    uint8_t ocr[4];
    spi_read_buf(ocr, 4);

    sd_send_cmd(SD_CMD16, SD_SECTOR_SIZE, 0x15);
    sd_cs_high();
    spi_txrx(SD_DUMMY_BYTE);
    spi_set_baudrate(spi0, SPI_FAST_FREQUENCY);

    card_initialized = true;
    mutex_exit(&spi_mutex);
    return 0;
}

DSTATUS disk_status(BYTE pdrv) {
    (void)pdrv;
    return card_initialized ? 0 : STA_NOINIT;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count) {
    (void)pdrv;
    if (!card_initialized) return RES_NOTRDY;

    mutex_enter_blocking(&spi_mutex);
    sd_send_cmd(SD_CMD17, sector, 0x01);
    bool ok = sd_receive_data_block(buff, SD_SECTOR_SIZE);
    sd_cs_high();
    spi_txrx(SD_DUMMY_BYTE);
    mutex_exit(&spi_mutex);

    return ok ? RES_OK : RES_ERROR;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count) {
    (void)pdrv;
    if (!card_initialized) return RES_NOTRDY;

    printf("Debug: disk_write sector=%lu, count=%u\n", (uint32_t)sector, count);

    mutex_enter_blocking(&spi_mutex);
    uint8_t resp = sd_send_cmd(SD_CMD24, sector, 0x01);
    printf("Debug: CMD24 resp=0x%02X\n", resp);

    bool ok = false;
    if (resp == 0x00)
        ok = sd_send_data_block(buff, SD_TOKEN_START_BLOCK);
    else
        printf("Debug: CMD24 failed\n");

    sd_cs_high();
    spi_txrx(SD_DUMMY_BYTE);
    mutex_exit(&spi_mutex);

    printf("Debug: disk_write result=%d\n", ok);
    return ok ? RES_OK : RES_ERROR;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff) {
    (void)pdrv;
    switch (cmd) {
        case CTRL_SYNC:
            return sd_wait_ready(500) ? RES_OK : RES_ERROR;
        case GET_SECTOR_SIZE:
            *(WORD *)buff = SD_SECTOR_SIZE;
            return RES_OK;
        case GET_BLOCK_SIZE:
            *(DWORD *)buff = 1;
            return RES_OK;
        case GET_SECTOR_COUNT:
            *(DWORD *)buff = 0;
            return RES_OK;
        default:
            return RES_PARERR;
    }
}
