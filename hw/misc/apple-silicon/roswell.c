/*
 * Apple Roswell (Authentication CP Relay IC).
 *
 * Copyright (c) 2023-2026 Visual Ehrmanntraut (VisualEhrmanntraut).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * Roswell is the board's Apple authentication coprocessor (the MFi "CP" part),
 * sitting on i2c3.  iOS binds `RoswellAuthI2CRelayInterface` (AppleAuthCP.kext,
 * `IONameMatch = roswell`) to it and stacks `AppleAuthCPRelay` on top.
 *
 * The wire protocol is the MFi coprocessor register file.  The master writes a
 * one byte register address, optionally followed by the data to write, and
 * then issues a separate read transfer to stream the register's contents back.
 * Registers are variable width; reads run on from wherever the last one
 * stopped, and only a new address write rewinds them.  That matters because
 * the driver pulls the certificate in 120 byte chunks without re-selecting the
 * register in between.
 *
 * Only the registers iOS actually touches are modelled.  Everything else reads
 * back as zeroes rather than NAKing: a NAK surfaces in the guest as
 * `AppleAuthCPRelay:_sendCommandGated ... authSendData() failed`, and the
 * driver cannot tell "no such register" from "the bus is broken".
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/misc/apple-silicon/roswell.h"
#include "migration/vmstate.h"
#include "qemu/log.h"

#if 0
#define ROSWELL_DPRINTF(v, ...)                                        \
    fprintf(stderr, "%s:%s@%d: " v "\n", __func__, __FILE__, __LINE__, \
            ##__VA_ARGS__)
#else
#define ROSWELL_DPRINTF(v, ...) \
    do {                        \
    } while (0)
#endif

/* MFi authentication coprocessor register map. */
enum {
    ROSWELL_REG_DEVICE_VERSION = 0x00,
    ROSWELL_REG_FIRMWARE_VERSION = 0x01,
    ROSWELL_REG_AUTH_MAJOR_VERSION = 0x02,
    ROSWELL_REG_AUTH_MINOR_VERSION = 0x03,
    ROSWELL_REG_DEVICE_ID = 0x04, /* 4 bytes */
    ROSWELL_REG_ERROR_CODE = 0x05,
    ROSWELL_REG_AUTH_CONTROL_STATUS = 0x10,
    ROSWELL_REG_SIGNATURE_LENGTH = 0x11, /* 2 bytes */
    ROSWELL_REG_SIGNATURE_DATA = 0x12, /* ROSWELL_SIGNATURE_LEN bytes */
    ROSWELL_REG_CHALLENGE_LENGTH = 0x20, /* 2 bytes */
    ROSWELL_REG_CHALLENGE_DATA = 0x21, /* ROSWELL_CHALLENGE_LEN bytes */
    ROSWELL_REG_CERT_LENGTH = 0x30, /* 2 bytes, big endian */
    ROSWELL_REG_CERT_DATA = 0x31, /* pages 0x31..0x3A */
    ROSWELL_REG_CERT_DATA_LAST = 0x3A,
    ROSWELL_REG_SELF_TEST_STATUS = 0x40,
    ROSWELL_REG_CERT_SERIAL_NUMBER = 0x4E, /* 32 bytes */
    ROSWELL_REG_EXTENDED = 0x60,
};

/* Authentication Control and Status. */
#define ROSWELL_AUTH_CONTROL_START_NEW_SIGNATURE 1
#define ROSWELL_AUTH_STATUS_SHIFT 4
#define ROSWELL_AUTH_STATUS_SIGNATURE_OK 0x1

/* One page of certificate data, per the coprocessor's paged register map. */
#define ROSWELL_CERT_PAGE_LEN 128

#define ROSWELL_CHALLENGE_LEN 32
#define ROSWELL_SIGNATURE_LEN 64

/*
 * The vendor specific window at ROSWELL_REG_EXTENDED.  The driver writes a
 * nine byte request and reads a seven byte reply: a status byte followed by
 * the coprocessor's six byte ID serial number, which it republishes as the
 * provider's IDSN.  (An older comment here claimed the marker byte lived at
 * offset 9; it does not -- the guest only ever reads seven bytes.)
 */
#define ROSWELL_EXTENDED_RESPONSE_LEN 7
#define ROSWELL_IDSN_LEN 6
#define ROSWELL_IDSN "CKRosw"

/*
 * The certificate the coprocessor hands out.
 *
 * On real hardware this is the Apple-issued MFi certificate burned into the
 * CP.  It cannot be forged, and there is nothing to fall back on: the register
 * file has no encoding for "there is no certificate".  A zero
 * ROSWELL_REG_CERT_LENGTH is a hardware fault as far as the driver is
 * concerned -- it turns into kIOReturnDeviceError -- which is what used to
 * send AppleAuthCPRelay into an unbounded retry loop, with userspace
 * re-reading the (never published) AccessoryCertificate property forever.
 *
 * So we hand out a self-signed placeholder instead.  Nothing in the guest can
 * verify it -- MFi verification happens on the far side of the handshake,
 * against Apple's root -- but it parses as X.509 and it lets the driver reach
 * a terminal state.
 *
 * The length is not free: AppleAuthCP's relay interface range-checks
 * ROSWELL_REG_CERT_LENGTH against its fixed 0x261 byte staging buffer and
 * rejects anything outside 607..609 bytes with "Certificate length not valid".
 * Hence exactly 609.  Regenerate with (needs OpenSSL 3; the OU padding tunes
 * the DER length, and the ECDSA signature length varies, so retry until the
 * output is exactly 609 bytes):
 *
 *   openssl req -x509 -newkey ec -pkeyopt ec_paramgen_curve:prime256v1 \
 *       -keyout /dev/null -nodes -days 7300 -sha256 -config gen.cnf \
 *       -outform DER -out cert.der
 */
static const uint8_t apple_roswell_certificate[] = {
    0x30, 0x82, 0x02, 0x5D, 0x30, 0x82, 0x02, 0x03, 0xA0, 0x03, 0x02, 0x01,
    0x02, 0x02, 0x14, 0x6A, 0x19, 0x09, 0x84, 0x79, 0xF4, 0x89, 0xBC, 0x0F,
    0x5E, 0xEB, 0x82, 0x51, 0x0C, 0xE8, 0xDC, 0x4F, 0x24, 0x82, 0xD8, 0x30,
    0x0A, 0x06, 0x08, 0x2A, 0x86, 0x48, 0xCE, 0x3D, 0x04, 0x03, 0x02, 0x30,
    0x81, 0x9C, 0x31, 0x18, 0x30, 0x16, 0x06, 0x03, 0x55, 0x04, 0x03, 0x0C,
    0x0F, 0x49, 0x6E, 0x66, 0x65, 0x72, 0x6E, 0x6F, 0x20, 0x52, 0x6F, 0x73,
    0x77, 0x65, 0x6C, 0x6C, 0x31, 0x45, 0x30, 0x43, 0x06, 0x03, 0x55, 0x04,
    0x0B, 0x0C, 0x3C, 0x45, 0x6D, 0x75, 0x6C, 0x61, 0x74, 0x65, 0x64, 0x20,
    0x41, 0x70, 0x70, 0x6C, 0x65, 0x20, 0x41, 0x75, 0x74, 0x68, 0x65, 0x6E,
    0x74, 0x69, 0x63, 0x61, 0x74, 0x69, 0x6F, 0x6E, 0x20, 0x43, 0x6F, 0x70,
    0x72, 0x6F, 0x63, 0x65, 0x73, 0x73, 0x6F, 0x72, 0x20, 0x2D, 0x20, 0x49,
    0x6E, 0x66, 0x65, 0x72, 0x6E, 0x6F, 0x20, 0x65, 0x6D, 0x75, 0x6C, 0x61,
    0x74, 0x6F, 0x72, 0x31, 0x39, 0x30, 0x37, 0x06, 0x03, 0x55, 0x04, 0x0B,
    0x0C, 0x30, 0x70, 0x6C, 0x61, 0x63, 0x65, 0x68, 0x6F, 0x6C, 0x64, 0x65,
    0x72, 0x20, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E,
    0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E,
    0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E,
    0x2E, 0x2E, 0x30, 0x1E, 0x17, 0x0D, 0x32, 0x36, 0x30, 0x37, 0x33, 0x31,
    0x31, 0x34, 0x30, 0x30, 0x34, 0x34, 0x5A, 0x17, 0x0D, 0x34, 0x36, 0x30,
    0x37, 0x32, 0x36, 0x31, 0x34, 0x30, 0x30, 0x34, 0x34, 0x5A, 0x30, 0x81,
    0x9C, 0x31, 0x18, 0x30, 0x16, 0x06, 0x03, 0x55, 0x04, 0x03, 0x0C, 0x0F,
    0x49, 0x6E, 0x66, 0x65, 0x72, 0x6E, 0x6F, 0x20, 0x52, 0x6F, 0x73, 0x77,
    0x65, 0x6C, 0x6C, 0x31, 0x45, 0x30, 0x43, 0x06, 0x03, 0x55, 0x04, 0x0B,
    0x0C, 0x3C, 0x45, 0x6D, 0x75, 0x6C, 0x61, 0x74, 0x65, 0x64, 0x20, 0x41,
    0x70, 0x70, 0x6C, 0x65, 0x20, 0x41, 0x75, 0x74, 0x68, 0x65, 0x6E, 0x74,
    0x69, 0x63, 0x61, 0x74, 0x69, 0x6F, 0x6E, 0x20, 0x43, 0x6F, 0x70, 0x72,
    0x6F, 0x63, 0x65, 0x73, 0x73, 0x6F, 0x72, 0x20, 0x2D, 0x20, 0x49, 0x6E,
    0x66, 0x65, 0x72, 0x6E, 0x6F, 0x20, 0x65, 0x6D, 0x75, 0x6C, 0x61, 0x74,
    0x6F, 0x72, 0x31, 0x39, 0x30, 0x37, 0x06, 0x03, 0x55, 0x04, 0x0B, 0x0C,
    0x30, 0x70, 0x6C, 0x61, 0x63, 0x65, 0x68, 0x6F, 0x6C, 0x64, 0x65, 0x72,
    0x20, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E,
    0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E,
    0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E,
    0x2E, 0x30, 0x59, 0x30, 0x13, 0x06, 0x07, 0x2A, 0x86, 0x48, 0xCE, 0x3D,
    0x02, 0x01, 0x06, 0x08, 0x2A, 0x86, 0x48, 0xCE, 0x3D, 0x03, 0x01, 0x07,
    0x03, 0x42, 0x00, 0x04, 0x45, 0x77, 0x46, 0x61, 0xE3, 0xB8, 0x8B, 0x98,
    0xB8, 0x6C, 0xEF, 0xFC, 0x08, 0xA7, 0x5A, 0x5E, 0xA5, 0x0F, 0xDE, 0x86,
    0x85, 0xAD, 0xB7, 0xB5, 0xDF, 0x9C, 0x04, 0xE6, 0x0E, 0x13, 0x4E, 0x9A,
    0xAE, 0x0F, 0x18, 0x1C, 0x39, 0x9E, 0x0E, 0xF4, 0x83, 0x13, 0x6A, 0xCE,
    0x8E, 0x75, 0x35, 0x4E, 0xFC, 0xA4, 0xC2, 0x48, 0x33, 0xCE, 0xCA, 0x7A,
    0xE5, 0x9F, 0x30, 0x2E, 0x7B, 0x82, 0x17, 0x03, 0xA3, 0x21, 0x30, 0x1F,
    0x30, 0x1D, 0x06, 0x03, 0x55, 0x1D, 0x0E, 0x04, 0x16, 0x04, 0x14, 0x9B,
    0xE8, 0xD1, 0x53, 0x8A, 0xC0, 0xB9, 0x69, 0xEB, 0x03, 0x75, 0x3A, 0xC7,
    0xC0, 0xE1, 0x01, 0xC5, 0x0D, 0x89, 0xDA, 0x30, 0x0A, 0x06, 0x08, 0x2A,
    0x86, 0x48, 0xCE, 0x3D, 0x04, 0x03, 0x02, 0x03, 0x48, 0x00, 0x30, 0x45,
    0x02, 0x20, 0x33, 0x60, 0x3C, 0x32, 0x88, 0x98, 0x7E, 0x71, 0xA6, 0x5D,
    0x78, 0x9F, 0xB9, 0xB4, 0x7C, 0x37, 0x41, 0x4C, 0x3B, 0xD9, 0x2B, 0x49,
    0xD6, 0xAF, 0x97, 0x02, 0xE3, 0x26, 0x60, 0xF3, 0x5F, 0xCD, 0x02, 0x21,
    0x00, 0xD0, 0x92, 0xBF, 0x4F, 0x20, 0xCB, 0xA7, 0x6B, 0xF0, 0x1D, 0x66,
    0xCE, 0x41, 0xF0, 0x1B, 0xC7, 0x92, 0xCF, 0xF5, 0xAD, 0x98, 0x23, 0xC5,
    0x9D, 0xC0, 0x09, 0xA7, 0xBE, 0xEB, 0x91, 0xA9, 0x6E,
};

/* SHA-256 of the certificate above, used as its 32-byte serial number. */
static const uint8_t apple_roswell_certificate_sn[] = {
    0xAA, 0x56, 0xCB, 0x46, 0x35, 0x01, 0x55, 0xED, 0x3A, 0x08, 0x7A, 0x58,
    0x1C, 0x17, 0xBA, 0xCB, 0x94, 0xD0, 0x9C, 0x71, 0x9B, 0x97, 0x3F, 0x67,
    0xF0, 0xFC, 0x2D, 0x6B, 0x13, 0x8D, 0x3E, 0x90,
};

typedef struct {
    uint8_t buf[sizeof(apple_roswell_certificate)];
    uint16_t len;
    uint16_t pos;
} AppleRoswellResponse;

struct AppleRoswellState {
    /*< private >*/
    I2CSlave i2c;

    /*< public >*/
    uint8_t reg;
    bool expect_reg;
    uint8_t error_code;
    uint8_t auth_control_status;
    uint8_t challenge[ROSWELL_CHALLENGE_LEN];
    uint16_t challenge_len;
    uint16_t write_pos;
    AppleRoswellResponse resp;
};

static void apple_roswell_resp_reset(AppleRoswellState *roswell)
{
    roswell->resp.len = 0;
    roswell->resp.pos = 0;
}

static void apple_roswell_resp_append(AppleRoswellState *roswell,
                                      const void *data, size_t len)
{
    size_t avail = sizeof(roswell->resp.buf) - roswell->resp.len;

    if (len > avail) {
        len = avail;
    }
    memcpy(&roswell->resp.buf[roswell->resp.len], data, len);
    roswell->resp.len += len;
}

static void apple_roswell_resp_append_u8(AppleRoswellState *roswell,
                                         uint8_t val)
{
    apple_roswell_resp_append(roswell, &val, sizeof(val));
}

static void apple_roswell_resp_append_be16(AppleRoswellState *roswell,
                                           uint16_t val)
{
    uint8_t buf[2];

    stw_be_p(buf, val);
    apple_roswell_resp_append(roswell, buf, sizeof(buf));
}

/*
 * The certificate is exposed through the paged registers 0x31..0x3A, one
 * ROSWELL_CERT_PAGE_LEN sized page each.  iOS reads the whole thing as one
 * run starting at 0x31, so reads simply carry on into the following pages.
 */
static void apple_roswell_append_cert_from(AppleRoswellState *roswell,
                                           uint8_t reg)
{
    size_t off = (size_t)(reg - ROSWELL_REG_CERT_DATA) * ROSWELL_CERT_PAGE_LEN;

    if (off >= sizeof(apple_roswell_certificate)) {
        return;
    }
    apple_roswell_resp_append(roswell, &apple_roswell_certificate[off],
                              sizeof(apple_roswell_certificate) - off);
}

/*
 * The signature over the last challenge.  A genuine one is made with the
 * coprocessor's Apple-issued private key, so there is nothing faithful to
 * compute; we return a deterministic stand-in derived from the challenge, so
 * that at least the same challenge always yields the same answer.
 */
static void apple_roswell_append_signature(AppleRoswellState *roswell)
{
    uint8_t sig[ROSWELL_SIGNATURE_LEN];
    size_t i;

    for (i = 0; i < sizeof(sig); i++) {
        sig[i] = roswell->challenge[i % ROSWELL_CHALLENGE_LEN] ^ (uint8_t)i;
    }
    apple_roswell_resp_append(roswell, sig, sizeof(sig));
}

static void apple_roswell_build_response(AppleRoswellState *roswell)
{
    uint8_t reg = roswell->reg;

    apple_roswell_resp_reset(roswell);

    if (reg >= ROSWELL_REG_CERT_DATA && reg <= ROSWELL_REG_CERT_DATA_LAST) {
        apple_roswell_append_cert_from(roswell, reg);
        return;
    }

    switch (reg) {
    case ROSWELL_REG_DEVICE_VERSION:
        apple_roswell_resp_append_u8(roswell, 0x00); /* Device Version */
        /* fallthrough */
    case ROSWELL_REG_FIRMWARE_VERSION:
        apple_roswell_resp_append_u8(roswell, 0x00); /* Firmware Version */
        /* fallthrough */
    case ROSWELL_REG_AUTH_MAJOR_VERSION:
        apple_roswell_resp_append_u8(roswell, 0x02); /* Auth Major Version */
        /* fallthrough */
    case ROSWELL_REG_AUTH_MINOR_VERSION:
        apple_roswell_resp_append_u8(roswell, 0x00); /* Auth Minor Version */
        /* fallthrough */
    case ROSWELL_REG_DEVICE_ID: {
        uint8_t id[4];

        stl_be_p(id, 0xDEADBEEF); /* Device ID */
        apple_roswell_resp_append(roswell, id, sizeof(id));
        break;
    }
    case ROSWELL_REG_ERROR_CODE:
        apple_roswell_resp_append_u8(roswell, roswell->error_code);
        break;
    case ROSWELL_REG_AUTH_CONTROL_STATUS:
        apple_roswell_resp_append_u8(roswell, roswell->auth_control_status);
        break;
    case ROSWELL_REG_SIGNATURE_LENGTH:
        apple_roswell_resp_append_be16(roswell, ROSWELL_SIGNATURE_LEN);
        break;
    case ROSWELL_REG_SIGNATURE_DATA:
        apple_roswell_append_signature(roswell);
        break;
    case ROSWELL_REG_CHALLENGE_LENGTH:
        apple_roswell_resp_append_be16(roswell, roswell->challenge_len);
        break;
    case ROSWELL_REG_CHALLENGE_DATA:
        apple_roswell_resp_append(roswell, roswell->challenge,
                                  sizeof(roswell->challenge));
        break;
    case ROSWELL_REG_CERT_LENGTH:
        apple_roswell_resp_append_be16(roswell,
                                       sizeof(apple_roswell_certificate));
        break;
    case ROSWELL_REG_SELF_TEST_STATUS:
        /* Certificate present and valid, private key present and valid. */
        apple_roswell_resp_append_u8(roswell, 0x50);
        break;
    case ROSWELL_REG_CERT_SERIAL_NUMBER:
        apple_roswell_resp_append(roswell, apple_roswell_certificate_sn,
                                  sizeof(apple_roswell_certificate_sn));
        break;
    case ROSWELL_REG_EXTENDED:
        apple_roswell_resp_append_u8(roswell, 0x00); /* status: no error */
        apple_roswell_resp_append(roswell, ROSWELL_IDSN, ROSWELL_IDSN_LEN);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "%s: read from unimplemented register 0x%02X\n",
                      __func__, reg);
        break;
    }
}

static void apple_roswell_write_reg(AppleRoswellState *roswell, uint8_t data)
{
    switch (roswell->reg) {
    case ROSWELL_REG_ERROR_CODE:
        roswell->error_code = data;
        break;
    case ROSWELL_REG_AUTH_CONTROL_STATUS:
        roswell->auth_control_status = data;
        if (data & ROSWELL_AUTH_CONTROL_START_NEW_SIGNATURE) {
            /* The signature is generated synchronously; report it ready. */
            roswell->auth_control_status = ROSWELL_AUTH_STATUS_SIGNATURE_OK
                                           << ROSWELL_AUTH_STATUS_SHIFT;
            roswell->error_code = 0;
        }
        break;
    case ROSWELL_REG_CHALLENGE_LENGTH:
        if (roswell->write_pos < 2) {
            roswell->challenge_len = (roswell->challenge_len << 8) | data;
        }
        break;
    case ROSWELL_REG_CHALLENGE_DATA:
        if (roswell->write_pos < sizeof(roswell->challenge)) {
            roswell->challenge[roswell->write_pos] = data;
        }
        break;
    default:
        /*
         * Writes to the remaining registers -- the vendor specific window at
         * 0x60 in particular -- carry no state we model.  Swallow them;
         * NAKing would make the guest think the bus itself failed.
         */
        break;
    }

    if (roswell->write_pos < UINT16_MAX) {
        roswell->write_pos++;
    }
}

static int apple_roswell_event(I2CSlave *s, enum i2c_event event)
{
    AppleRoswellState *roswell = APPLE_ROSWELL(s);

    ROSWELL_DPRINTF("event %d", event);

    switch (event) {
    case I2C_START_SEND:
    case I2C_START_SEND_ASYNC:
        /* The first byte of a write is the register address. */
        roswell->expect_reg = true;
        roswell->write_pos = 0;
        break;
    default:
        /*
         * Reads carry on from wherever the previous one stopped, so there is
         * deliberately nothing to do for I2C_START_RECV: only writing a
         * register address rewinds the stream.
         */
        break;
    }

    return 0;
}

static uint8_t apple_roswell_rx(I2CSlave *s)
{
    AppleRoswellState *roswell = APPLE_ROSWELL(s);
    uint8_t ret;

    if (roswell->resp.pos >= roswell->resp.len) {
        /*
         * Past the end of the register.  Real silicon keeps clocking out
         * whatever is on the bus; returning zeroes is harmless and, more to
         * the point, does not wedge us for the next transfer.
         */
        return 0x00;
    }

    ret = roswell->resp.buf[roswell->resp.pos++];
    ROSWELL_DPRINTF("reg 0x%02X -> 0x%02X", roswell->reg, ret);

    return ret;
}

static int apple_roswell_tx(I2CSlave *s, uint8_t data)
{
    AppleRoswellState *roswell = APPLE_ROSWELL(s);

    ROSWELL_DPRINTF("0x%02X", data);

    if (roswell->expect_reg) {
        roswell->reg = data;
        roswell->expect_reg = false;
        roswell->write_pos = 0;
        if (roswell->reg == ROSWELL_REG_CHALLENGE_LENGTH) {
            roswell->challenge_len = 0;
        }
        apple_roswell_build_response(roswell);
        ROSWELL_DPRINTF("select register 0x%02X", roswell->reg);
        return 0;
    }

    apple_roswell_write_reg(roswell, data);

    return 0;
}

static const VMStateDescription vmstate_apple_roswell = {
    .name = "AppleRoswellState",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields =
        (const VMStateField[]){
            VMSTATE_I2C_SLAVE(i2c, AppleRoswellState),
            VMSTATE_UINT8(reg, AppleRoswellState),
            VMSTATE_BOOL(expect_reg, AppleRoswellState),
            VMSTATE_UINT8(error_code, AppleRoswellState),
            VMSTATE_UINT8(auth_control_status, AppleRoswellState),
            VMSTATE_UINT8_ARRAY(challenge, AppleRoswellState,
                                ROSWELL_CHALLENGE_LEN),
            VMSTATE_UINT16(challenge_len, AppleRoswellState),
            VMSTATE_UINT16(write_pos, AppleRoswellState),
            VMSTATE_UINT16(resp.len, AppleRoswellState),
            VMSTATE_UINT16(resp.pos, AppleRoswellState),
            VMSTATE_UINT8_ARRAY(resp.buf, AppleRoswellState,
                                sizeof(apple_roswell_certificate)),
            VMSTATE_END_OF_LIST(),
        },
};

static void apple_roswell_reset_enter(Object *obj, ResetType type)
{
    AppleRoswellState *roswell = APPLE_ROSWELL(obj);

    roswell->reg = ROSWELL_REG_DEVICE_VERSION;
    roswell->expect_reg = true;
    roswell->error_code = 0;
    roswell->auth_control_status = 0;
    roswell->challenge_len = 0;
    roswell->write_pos = 0;
    memset(roswell->challenge, 0, sizeof(roswell->challenge));
    apple_roswell_resp_reset(roswell);
}

static void apple_roswell_class_init(ObjectClass *klass, const void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *c = I2C_SLAVE_CLASS(klass);

    rc->phases.enter = apple_roswell_reset_enter;

    dc->desc = "Apple Roswell";
    dc->user_creatable = false;
    dc->vmsd = &vmstate_apple_roswell;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);

    c->event = apple_roswell_event;
    c->recv = apple_roswell_rx;
    c->send = apple_roswell_tx;
}

static const TypeInfo apple_roswell_type_info = {
    .name = TYPE_APPLE_ROSWELL,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(AppleRoswellState),
    .class_init = apple_roswell_class_init,
};

static void apple_roswell_register_types(void)
{
    type_register_static(&apple_roswell_type_info);
}

type_init(apple_roswell_register_types);
