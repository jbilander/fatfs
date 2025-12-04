/*
 * diskio.c - FatFs low-level disk I/O layer for RP2350 (spi0)
 * Production version: SDHC-aware, single-block R/W
 */

#include "main.h"
#include "ff.h"
#include "diskio.h"

#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <string.h>
#include <stdint.h>

/* SD commands */
#define CMD0    (0x40+0)
#define CMD8    (0x40+8)
#define CMD16   (0x40+16)
#define CMD17   (0x40+17)
#define CMD24   (0x40+24)
#define CMD55   (0x40+55)
#define CMD58   (0x40+58)
#define ACMD41  (0x40+41)

#define SD_TOKEN_START_BLOCK 0xFE
#define SD_DUMMY_BYTE 0xFF
#define SD_SECTOR_SIZE 512

static bool card_initialized = false;
static bool sd_is_sdhc = false;

/* Helpers */
static inline void sd_cs_low(void)  { gpio_put(PIN_SS, 0); }
static inline void sd_cs_high(void) { gpio_put(PIN_SS, 1); }

static uint8_t spi_xfer(uint8_t b) {
    uint8_t r;
    spi_write_read_blocking(spi0, &b, &r, 1);
    return r;
}

static void spi_write_buf(const uint8_t *buf, size_t len) {
    spi_write_blocking(spi0, buf, len);
}

static void spi_read_buf(uint8_t *buf, size_t len) {
    spi_read_blocking(spi0, SD_DUMMY_BYTE, buf, len);
}

static void spi_clock_dummy(uint32_t n) {
    for (uint32_t i = 0; i < n; ++i) spi_xfer(SD_DUMMY_BYTE);
}

/* Send command and return R1 response (or 0xFF on timeout) */
static uint8_t sd_send_cmd(uint8_t cmd, uint32_t arg, uint8_t crc) {
    uint8_t out[6];
    out[0] = cmd;
    out[1] = (uint8_t)(arg >> 24);
    out[2] = (uint8_t)(arg >> 16);
    out[3] = (uint8_t)(arg >> 8);
    out[4] = (uint8_t)arg;
    out[5] = crc;

    // Ensure at least one idle byte before CS low
    sd_cs_high(); spi_xfer(SD_DUMMY_BYTE);
    sd_cs_low();  spi_xfer(SD_DUMMY_BYTE);

    spi_write_buf(out, 6);

    // Wait for response (bit7==0)
    uint8_t resp = 0xFF;
    for (int i = 0; i < 10; ++i) {
        resp = spi_xfer(SD_DUMMY_BYTE);
        if ((resp & 0x80) == 0) break;
    }
    return resp;
}

/* Wait until card returns 0xFF or timeout (ms) */
static bool sd_wait_ready(uint32_t timeout_ms) {
    absolute_time_t end = make_timeout_time_ms(timeout_ms);
    uint8_t v;
    do {
        v = spi_xfer(SD_DUMMY_BYTE);
        if (v == 0xFF) return true;
    } while (!time_reached(end));
    return false;
}

/* Receive single data block into buffer */
static bool sd_receive_block(uint8_t *buf) {
    // wait for data token
    absolute_time_t end = make_timeout_time_ms(200);
    uint8_t token;
    do {
        token = spi_xfer(SD_DUMMY_BYTE);
        if (token != 0xFF) break;
    } while (!time_reached(end));

    if (token != SD_TOKEN_START_BLOCK) return false;

    spi_read_buf(buf, SD_SECTOR_SIZE);
    // discard CRC
    spi_xfer(SD_DUMMY_BYTE);
    spi_xfer(SD_DUMMY_BYTE);
    return true;
}

/* Send a single data block from buffer, with token */
static bool sd_send_block(const uint8_t *buf, uint8_t token) {
    if (!sd_wait_ready(500)) return false;
    spi_xfer(token);
    spi_write_buf(buf, SD_SECTOR_SIZE);
    // dummy CRC
    spi_xfer(SD_DUMMY_BYTE);
    spi_xfer(SD_DUMMY_BYTE);

    uint8_t resp = spi_xfer(SD_DUMMY_BYTE);
    if ((resp & 0x1F) != 0x05) return false;
    // wait until not busy
    if (!sd_wait_ready(500)) return false;
    return true;
}

/*------------------------------------------------------------------------*/
/* FatFs required functions                                               */
/*------------------------------------------------------------------------*/

DSTATUS disk_initialize(BYTE pdrv) {
    (void)pdrv;

    spi_set_baudrate(spi0, SPI_SLOW_FREQUENCY);

    sd_cs_high();
    spi_clock_dummy(10);

    uint8_t r;
    int i;
    // CMD0: reset
    i = 1000;
    do {
        r = sd_send_cmd(CMD0, 0, 0x95);
        if (r == 1) break;
        sleep_ms(1);
    } while (--i);

    if (r != 1) {
        card_initialized = false;
        return STA_NOINIT;
    }

    // CMD8 to check voltage and supply pattern (if supported)
    r = sd_send_cmd(CMD8, 0x1AA, 0x87);
    if (r == 1) {
        uint8_t buf[4];
        spi_read_buf(buf, 4);
        // if echo is correct continue; otherwise keep going
        // then ACMD41 until ready
        i = 1000;
        do {
            sd_send_cmd(CMD55, 0, 0x65);
            r = sd_send_cmd(ACMD41, 1UL << 30, 0x77);
            if (r == 0) break;
            sleep_ms(1);
        } while (--i);
        // CMD58 read OCR
        if (sd_send_cmd(CMD58, 0, 0) == 0) {
            uint8_t ocr[4];
            spi_read_buf(ocr, 4);
            sd_is_sdhc = (ocr[0] & 0x40) != 0;
        }
    } else {
        // Older card flow: try ACMD41 without HCS
        i = 1000;
        do {
            sd_send_cmd(CMD55, 0, 0x65);
            r = sd_send_cmd(ACMD41, 0, 0x77);
            if (r == 0) break;
            sleep_ms(1);
        } while (--i);
        sd_is_sdhc = false;
    }

    if (!sd_is_sdhc) {
        sd_send_cmd(CMD16, SD_SECTOR_SIZE, 0x15);
    }

    sd_cs_high();
    spi_xfer(SD_DUMMY_BYTE);
    spi_set_baudrate(spi0, SPI_FAST_FREQUENCY);

    card_initialized = true;
    return 0;
}

DSTATUS disk_status(BYTE pdrv) {
    (void)pdrv;
    return card_initialized ? 0 : STA_NOINIT;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count) {
    (void)pdrv;
    if (!card_initialized) return RES_NOTRDY;

    if (!sd_is_sdhc) sector *= SD_SECTOR_SIZE;

    for (UINT i = 0; i < count; ++i) {
        if (sd_send_cmd(CMD17, (uint32_t)(sector + i), 0x01) != 0) {
            sd_cs_high();
            return RES_ERROR;
        }
        if (!sd_receive_block(buff + i * SD_SECTOR_SIZE)) {
            sd_cs_high();
            return RES_ERROR;
        }
        sd_cs_high();
        spi_xfer(SD_DUMMY_BYTE);
    }

    return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count) {
    (void)pdrv;
    if (!card_initialized) return RES_NOTRDY;

    if (!sd_is_sdhc) sector *= SD_SECTOR_SIZE;

    for (UINT i = 0; i < count; ++i) {
        if (sd_send_cmd(CMD24, (uint32_t)(sector + i), 0x01) != 0) {
            sd_cs_high();
            return RES_ERROR;
        }
        if (!sd_send_block(buff + i * SD_SECTOR_SIZE, SD_TOKEN_START_BLOCK)) {
            sd_cs_high();
            return RES_ERROR;
        }
        sd_cs_high();
        spi_xfer(SD_DUMMY_BYTE);
    }

    return RES_OK;
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
            *(DWORD *)buff = 0; // optional
            return RES_OK;
        default:
            return RES_PARERR;
    }
}