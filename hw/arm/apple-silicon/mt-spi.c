/*
 * Apple Multi Touch SPI Controller.
 *
 * Copyright (c) 2025-2026 Visual Ehrmanntraut (VisualEhrmanntraut).
 * Copyright (c) 2025-2026 Christian Inci (chris-pcguy).
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
 * The touch controller speaks two protocols over the same SPI chip select.
 *
 * Before it has been booted it speaks HBPP, a small request/response protocol
 * the host uses to push firmware into the controller's memory. Once running it
 * speaks HID-over-SPI: a link layer carrying fixed-size frames, inside which
 * sit HID control requests and HID reports.
 *
 * The frame geometry and the two statistics reports below are not guesswork.
 * iOS loads a firmware personality into this controller, and that personality
 * is a plist we can read: the guest device tree node arm-io/spi1/multi-touch
 * carries
 *
 *     hid-merge-personality = "C1FC2,2"
 *
 * which names a dictionary inside N104_Multitouch.im4p (the IM4P payload is an
 * OSSerialize XML plist in the clear). Its "Config" dictionary configures
 * AppleHIDTransportProtocolHIDSPI and states, verbatim:
 *
 *     HIDSPI Config    = { Transfer Length = 512, Padding Length = 4,
 *                          Max Input Report Length = 4096,
 *                          Device Interface Id = 208, ... }
 *     Interface Config = [ { bInterfaceNumber = 0, InterfaceName = "grape",
 *                            InterfaceType = "HID",
 *                            DefaultMultitouchProperties = { ExpectedVersion
 *                                = 658, ... },
 *                            Reporters = [ ... ] } ]
 *
 * "Padding Length" + "Transfer Length" is where the 516-byte frame comes from,
 * and the "Reporters" array is where the two statistics reports this device
 * has to answer (0xF8 and 0x72) get their report IDs and lengths. Both are
 * quoted again at their implementations below.
 */

#include "qemu/osdep.h"
#include "block/aio.h"
#include "hw/arm/apple-silicon/mt-spi.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/ssi/ssi.h"
#include "migration/vmstate.h"
#include "qemu/bswap.h"
#include "qemu/crc16.h"
#include "qemu/error-report.h"
#include "qemu/lockable.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qemu/timer.h"
#include "ui/console.h"
#include "ui/input.h"

typedef struct AppleMTSPIBuffer {
    uint8_t *data;
    uint32_t capacity;
    uint32_t len;
    uint32_t read_pos;
} AppleMTSPIBuffer;

static const VMStateDescription vmstate_apple_mt_spi_buffer = {
    .name = "AppleMTSPIBuffer",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields =
        (const VMStateField[]){
            VMSTATE_UINT32(capacity, AppleMTSPIBuffer),
            VMSTATE_VBUFFER_ALLOC_UINT32(data, AppleMTSPIBuffer, 0, NULL,
                                         capacity),
            VMSTATE_UINT32(len, AppleMTSPIBuffer),
            VMSTATE_UINT32(read_pos, AppleMTSPIBuffer),
            VMSTATE_END_OF_LIST(),
        },
};

typedef struct AppleMTSPILLPacket {
    AppleMTSPIBuffer buf;
    uint8_t type;
    /// Link-layer interface this packet belongs to. Responses echo the
    /// interface their request arrived on; unsolicited reports use the HID
    /// interface, which the personality numbers 0.
    uint8_t interface;
    QTAILQ_ENTRY(AppleMTSPILLPacket) next;
} AppleMTSPILLPacket;

static const VMStateDescription vmstate_apple_mt_spi_ll_packet = {
    .name = "AppleMTSPILLPacket",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields =
        (const VMStateField[]){
            VMSTATE_STRUCT(buf, AppleMTSPILLPacket, 0,
                           vmstate_apple_mt_spi_buffer, AppleMTSPIBuffer),
            VMSTATE_UINT8(type, AppleMTSPILLPacket),
            VMSTATE_UINT8(interface, AppleMTSPILLPacket),
            VMSTATE_END_OF_LIST(),
        },
};

/// Counters behind HID report 0xF8, the transport statistics. The host reads
/// them and then clears them by writing the same report back, so they count
/// events since the last clear rather than since power-on (the personality
/// pairs ReportID 248 with ClearReportID 248 and sets AbsoluteValues = false,
/// meaning the host adds what it reads to its own running totals).
typedef struct AppleMTSPIStats {
    uint32_t get_reports;
    uint32_t set_reports;
    uint32_t input_reports;
    uint32_t input_drops;
    uint32_t output_reports;
} AppleMTSPIStats;

static const VMStateDescription vmstate_apple_mt_spi_stats = {
    .name = "AppleMTSPIStats",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields =
        (const VMStateField[]){
            VMSTATE_UINT32(get_reports, AppleMTSPIStats),
            VMSTATE_UINT32(set_reports, AppleMTSPIStats),
            VMSTATE_UINT32(input_reports, AppleMTSPIStats),
            VMSTATE_UINT32(input_drops, AppleMTSPIStats),
            VMSTATE_UINT32(output_reports, AppleMTSPIStats),
            VMSTATE_END_OF_LIST(),
        },
};

struct AppleMTSPIState {
    SSIPeripheral parent_obj;

    QemuMutex lock;
    /// IRQ of the Multi Touch Controller is Active Low.
    /// qemu_irq_raise means IRQ inactive,
    /// qemu_irq_lower means IRQ active.
    qemu_irq irq;
    AppleMTSPIBuffer tx;
    AppleMTSPIBuffer rx;
    AppleMTSPIBuffer pending_hbpp;
    QTAILQ_HEAD(, AppleMTSPILLPacket) pending_fw;
    uint8_t frame;
    QEMUTimer *timer;
    QEMUTimer *end_timer;
    int16_t x;
    int16_t y;
    int16_t prev_x;
    int16_t prev_y;
    uint64_t prev_ts;
    int32_t btn_state;
    int32_t prev_btn_state;
    uint32_t display_width;
    uint32_t display_height;
    AppleMTSPIStats stats;
    /// Virtual-clock time at which the power statistics were last cleared,
    /// i.e. the start of the interval HID report 0x72 describes.
    uint64_t power_stats_since;
};

static const VMStateDescription vmstate_apple_mt_spi = {
    .name = "AppleMTSPIState",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields =
        (const VMStateField[]){
            VMSTATE_SSI_PERIPHERAL(parent_obj, AppleMTSPIState),
            VMSTATE_STRUCT(tx, AppleMTSPIState, 0, vmstate_apple_mt_spi_buffer,
                           AppleMTSPIBuffer),
            VMSTATE_STRUCT(rx, AppleMTSPIState, 0, vmstate_apple_mt_spi_buffer,
                           AppleMTSPIBuffer),
            VMSTATE_STRUCT(pending_hbpp, AppleMTSPIState, 0,
                           vmstate_apple_mt_spi_buffer, AppleMTSPIBuffer),
            VMSTATE_QTAILQ_V(pending_fw, AppleMTSPIState, 0,
                             vmstate_apple_mt_spi_ll_packet, AppleMTSPILLPacket,
                             next),
            VMSTATE_UINT8(frame, AppleMTSPIState),
            VMSTATE_TIMER_PTR(timer, AppleMTSPIState),
            VMSTATE_TIMER_PTR(end_timer, AppleMTSPIState),
            VMSTATE_INT16(x, AppleMTSPIState),
            VMSTATE_INT16(y, AppleMTSPIState),
            VMSTATE_INT16(prev_x, AppleMTSPIState),
            VMSTATE_INT16(prev_y, AppleMTSPIState),
            VMSTATE_UINT64(prev_ts, AppleMTSPIState),
            VMSTATE_INT32(btn_state, AppleMTSPIState),
            VMSTATE_INT32(prev_btn_state, AppleMTSPIState),
            VMSTATE_UINT32(display_width, AppleMTSPIState),
            VMSTATE_UINT32(display_height, AppleMTSPIState),
            VMSTATE_STRUCT(stats, AppleMTSPIState, 0, vmstate_apple_mt_spi_stats,
                           AppleMTSPIStats),
            VMSTATE_UINT64(power_stats_since, AppleMTSPIState),
            VMSTATE_END_OF_LIST(),
        },
};

// HBPP Command:
// u8 packet_type
// u1 unk0
// u3 packet_size? // 0 = Empty
// u4 unk1

#define HBPP_PACKET_RESET (0x00)
#define HBPP_PACKET_NOP (0x18)
#define HBPP_PACKET_INT_ACK (0x1A)
#define HBPP_PACKET_MEM_READ (0x1C)
#define HBPP_PACKET_MEM_RMW (0x1E)
#define HBPP_PACKET_REQ_CAL (0x1F)
#define HBPP_PACKET_DATA (0x30)

#define HBPP_PACKET_REQ_BOOT (0x011F)
#define HBPP_PACKET_ACK_RD_REQ (0x394C)
#define HBPP_PACKET_ACK_NOP (0x7948)
#define HBPP_PACKET_CAL_DONE (0x6949)
#define HBPP_PACKET_ACK_DATA (0xC14B)
#define HBPP_PACKET_ACK_WR_REQ (0xD14A)

/// Every frame on the wire is a 4-byte preamble followed by 512 bytes of
/// framed data. Both halves are named by the firmware personality's
/// "HIDSPI Config" dictionary, as "Padding Length" (4) and "Transfer Length"
/// (512). The preamble word is materialised by
/// AppleHIDTransportProtocolHIDSPI::cacheProtocolConfig().
#define LL_PACKET_PREAMBLE (0xDEADBEEF)
#define LL_PADDING_LEN (4)
#define LL_TRANSFER_LEN (512)
#define LL_PACKET_LEN (LL_PADDING_LEN + LL_TRANSFER_LEN)

/// The link-layer interface the touch reports belong to. The personality's
/// sole "Interface Config" entry has bInterfaceNumber = 0.
#define LL_INTERFACE_HID (0)
/// Frames addressed to the controller itself rather than to one of its HID
/// interfaces, as "HIDSPI Config"'s "Device Interface Id" = 208.
#define LL_INTERFACE_DEVICE (0xD0)

/*
 * Link-layer frame types. "Input" and "Output" are named for the direction the
 * data leaves the sender, so the controller's own touch reports are sent as
 * OUTPUT and the frames iOS sends down are OUTPUT too. "Lossless" frames are
 * the ones the sender would retry if the receiver reported itself busy.
 */
#define LL_PACKET_LOSSLESS_OUTPUT (0x10)
#define LL_PACKET_LOSSY_OUTPUT (0x11)
#define LL_PACKET_LOSSLESS_INPUT (0x20)
#define LL_PACKET_LOSSY_INPUT (0x21)
#define LL_PACKET_CONTROL (0x40)
#define LL_PACKET_NO_DATA (0x80)

#define LL_PACKET_ERROR (0xB3655245)
#define LL_PACKET_ACK (0xD56827AC)
#define LL_PACKET_NAK (0xE4FB139)
#define LL_PACKET_BUSY (0xF8E5179C)

#define HID_CONTROL_PACKET_GET_RESULT_DATA (0x10)
#define HID_CONTROL_PACKET_SET_RESULT_DATA (0x20)
#define HID_CONTROL_PACKET_GET_INPUT_REPORT (0x30)
#define HID_CONTROL_PACKET_GET_OUTPUT_REPORT (0x31)
#define HID_CONTROL_PACKET_GET_FEATURE_REPORT (0x32)
#define HID_CONTROL_PACKET_SET_INPUT_REPORT (0x50)
#define HID_CONTROL_PACKET_SET_OUTPUT_REPORT (0x51)
#define HID_CONTROL_PACKET_SET_FEATURE_REPORT (0x52)

#define HID_TRANSFER_PACKET_INPUT (0x10)
#define HID_TRANSFER_PACKET_OUTPUT (0x20)

#define HID_PACKET_STATUS_SUCCESS (0)
#define HID_PACKET_STATUS_BUSY (1)
#define HID_PACKET_STATUS_ERROR (2)
#define HID_PACKET_STATUS_ERROR_ID_MISMATCH (3)
#define HID_PACKET_STATUS_ERROR_UNSUPPORTED (4)
#define HID_PACKET_STATUS_ERROR_INCORRECT_LENGTH (5)

#define HID_REPORT_BINARY_PATH_OR_IMAGE (0x44)
#define HID_REPORT_POWER_STATS (0x72)
#define HID_REPORT_POWER_STATS_DESC (0x73)
#define HID_REPORT_STATUS (0x7F)
#define HID_REPORT_SENSOR_REGION_PARAM (0xA1)
#define HID_REPORT_SENSOR_REGION_DESC (0xD0)
#define HID_REPORT_FAMILY_ID (0xD1)
#define HID_REPORT_BASIC_DEVICE_INFO (0xD3)
#define HID_REPORT_BUTTONS (0xD7)
#define HID_REPORT_SENSOR_SURFACE_DESC (0xD9)
#define HID_REPORT_TRANSPORT_STATS (0xF8)

// maybe 0xc2?
#define MT_FAMILY_ID (0xC3)
#define MT_LITTLE_ENDIAN (1)
#define MT_ROWS (31)
#define MT_COLUMNS (15)
/// The interface version the firmware personality expects, as
/// DefaultMultitouchProperties["ExpectedVersion"] = 658 = 0x292. It goes on
/// the wire big-endian, hence the swap.
#define MT_BCD_VER (bswap16(0x292))
/*
 * The sensor surface is described in hundredths of a millimetre: the iPhone 11
 * panel is 326 ppi, so one display pixel spans 25.4/326 mm = 7.79 of those
 * units, and its 828 x 1792 pixels come out as the panel's 64.6 mm x 139.8 mm
 * active area.
 *
 * These are deliberately left as constants rather than derived from the
 * display_width/display_height properties: the figures below, together with
 * the y-axis offset applied in apple_mt_spi_mouse_event(), were calibrated by
 * hand against a 828 x 1792 panel to land within a pixel, and recomputing them
 * from a rounded units-per-pixel ratio moves them by enough to lose that.
 */
#define MT_SENSOR_SURFACE_WIDTH (6458) // 828 px * 7.8
#define MT_SENSOR_SURFACE_HEIGHT (13977) // 1792 px * 7.8

#define PATH_STAGE_NOT_TRACKING (0)
#define PATH_STAGE_START_IN_RANGE (1)
#define PATH_STAGE_HOVER_IN_RANGE (2)
#define PATH_STAGE_MAKE_TOUCH (3)
#define PATH_STAGE_TOUCHING (4)
#define PATH_STAGE_BREAK_TOUCH (5)
#define PATH_STAGE_LINGER_IN_RANGE (6)
#define PATH_STAGE_OUT_OF_RANGE (7)

/// Link-layer frame header, immediately after the preamble.
typedef struct {
    uint8_t type;
    uint8_t interface;
    /// Offset of this fragment within the payload being reassembled.
    uint16_t payload_off;
    /// Payload bytes still to come after this fragment.
    uint16_t payload_remaining;
    /// Payload bytes carried by this fragment.
    uint16_t payload_length;
} QEMU_PACKED AppleMTSPILLHeader;

/// Header of the HID request or report carried inside a link-layer payload.
typedef struct {
    uint8_t type;
    uint8_t report_id;
    uint8_t status;
    uint8_t frame_number;
    uint16_t length_requested;
    uint16_t payload_length;
} QEMU_PACKED AppleMTSPIHIDHeader;

/// Header of a HID report 0x44 frame, which carries the tracked contacts.
typedef struct {
    uint8_t frame_number;
    /// Offset from the start of the report at which the path array begins, so
    /// the parser can skip header fields it does not know. It is measured from
    /// the report ID byte, which sits at report offset 0 ahead of this struct,
    /// so it is one more than sizeof(AppleMTSPIFrameHeader).
    uint8_t header_len;
    uint8_t reserved0;
    /// Milliseconds since the controller started scanning.
    uint32_t timestamp;
    uint8_t reserved1[4];
    uint16_t reserved2;
    /// Length of the raw sensor image following the paths, if any.
    uint16_t image_len;
    uint8_t path_count;
    /// Size of one path entry, so the parser can stride over entries carrying
    /// fields it does not know.
    uint8_t path_len;
    uint8_t reserved3[10];
} QEMU_PACKED AppleMTSPIFrameHeader;

/// One tracked contact.
typedef struct {
    uint8_t id;
    uint8_t stage;
    uint8_t finger_id;
    uint8_t hand_id;
    /// Position on the sensor surface, in the units of the surface descriptor.
    int16_t x;
    int16_t y;
    /// Velocity, in surface units per second.
    int16_t vel_x;
    int16_t vel_y;
    /// Contact ellipse. iOS derives the contact density from these, so a
    /// plausible fingertip size keeps it out of its "implausible contact"
    /// paths.
    uint16_t radius_major;
    uint16_t radius_minor;
    uint16_t orientation;
    uint16_t radius_scale;
} QEMU_PACKED AppleMTSPIPath;

/// Payload bytes that fit in one link-layer frame, after its header and the
/// trailing CRC.
#define LL_PAYLOAD_MAX \
    (LL_TRANSFER_LEN - sizeof(AppleMTSPILLHeader) - sizeof(uint16_t))

/*
 * HID report 0xF8, the transport statistics. The host's reporter definition
 * lives in the firmware personality:
 *
 *   { Type = "Simple", ReportID = 248, ReportLength = 449,
 *     ClearReportID = 248, ClearReportZeroPad = true, AbsoluteValues = false,
 *     GroupName = "multi-touch", SubgroupName = "touch",
 *     Channels = [ { Name = "touch.transport.get-report-count",
 *                    Type = "Counter", Size = 4, Offset = 1 }, ... ] }
 *
 * The report must be at least ReportLength bytes or AIDReporterSimple::
 * updateReportWithData() rejects it outright, logging
 *
 *   [AIDReporter::updateReportWithData]: ERROR!! data->getLength() >=
 *   _dataLength ... AIDReporterSimple.cpp, line: 128
 *
 * which is what this device used to do on every boot. The report therefore has
 * to be the full 449 bytes even though only the first 321 carry channels: the
 * highest channel sits at offset 317 with size 4. Offset 0 is the report ID,
 * which is why every channel offset is odd. The named counters below are the
 * only ones this controller has anything truthful to say about; the remaining
 * channels are timing histograms of a scan engine that does not exist here, and
 * they stay zero.
 */
#define MT_TRANSPORT_STATS_LEN (449)
#define MT_STATS_OFF_GET_REPORT_COUNT (1)
#define MT_STATS_OFF_SET_REPORT_COUNT (5)
#define MT_STATS_OFF_INPUT_REPORT_COUNT (9)
#define MT_STATS_OFF_INPUT_DROP_COUNT (13)
#define MT_STATS_OFF_OUTPUT_REPORT_COUNT (17)
#define MT_STATS_OFF_NO_PIPELINE_ITEM (21)

/*
 * HID report 0x72, the power state residency. Its reporter definition is the
 * second entry of the same "Reporters" array:
 *
 *   { Type = "State", ReportID = 114, NumberOfStates = 11,
 *     GroupName = "Multitouch", SubgroupName = "Multitouch high level stats",
 *     Unit = "Us", ClearReportID = 114, ClearReportZeroPad = true,
 *     Channels = [ { Name = "High Level", Offset = 1, TicksPerSecond = 1000,
 *                    States = [ "Unknown", "Active", "TTW", "TTWSup", "Rsv",
 *                               "FaceDet", "Rsv", "Rsv",
 *                               "Active-Untriggered", "Rsv", "Diag" ] } ] }
 *
 * AIDReporterState::Channel::updateReportWithData() reads the channel as
 * NumberOfStates records of eight bytes each - a u32 tick count followed by a
 * u32 entry count - starting at Offset, so the report must hold
 * Offset + NumberOfStates * 8 bytes. Short of that the host cannot map the
 * channel and logs
 *
 *   [AIDReporterState::Channel::updateReportWithData]: ERROR!! statesData
 *   ... AIDReporterState.cpp, line: 377
 */
#define MT_POWER_STATS_NUM_STATES (11)
#define MT_POWER_STATS_OFF_HIGH_LEVEL (1)
#define MT_POWER_STATS_RECORD_LEN (8)
#define MT_POWER_STATS_TICKS_PER_SECOND (1000)
#define MT_POWER_STATS_LEN                \
    (MT_POWER_STATS_OFF_HIGH_LEVEL +      \
     MT_POWER_STATS_NUM_STATES * MT_POWER_STATS_RECORD_LEN)
/// Index into the "High Level" state list above. The emulated panel is always
/// scanning, so all of its residency is reported against "Active".
#define MT_POWER_STATE_ACTIVE (1)

/// HID report 0x7F latches the controller's critical errors.
/// AppleMultitouchDevice::decodeDeviceProperty() reads it as a little-endian
/// u16 when the payload is exactly two bytes and a u32 when it is exactly
/// four; every other length decodes as zero.
#define MT_STATUS_LEN (4)

/*
 * Two reports are deliberately left unimplemented, and both degrade quietly.
 *
 * 0xDB is a bulk device-properties blob that AppleMultitouchDevice::
 * _cacheDeviceProperties() tries first: a version byte, then TLVs carrying the
 * individual reports. When it does not parse, the driver falls back to
 * fetching { 0xD1, 0xD3, 0xD0, 0xA1, 0xD9, 0x7F } one at a time, which is
 * exactly the set answered above, so implementing it would only add a second
 * way to get the same answers wrong.
 *
 * 0x73 is the descriptor for the power statistics, and declining it is a
 * deliberate choice rather than an omission. AppleMultitouchPowerStats::
 * _parseDescriptor() rejects a descriptor whose version byte is not 1 and then
 * disables the whole subsystem without logging at the default level - which is
 * why no power-stats complaint appears in a boot log. Were it accepted, the
 * driver would become a second consumer of report 0x72 with its own length
 * expectation, derived from the descriptor rather than from the personality
 * that the AID reporter uses, and its state-id table only names six states
 * against the reporter's eleven. Serving one consumer correctly beats serving
 * two inconsistently.
 */

static inline void apple_mt_spi_buf_free(AppleMTSPIBuffer *buf)
{
    g_free(buf->data);
    *buf = (AppleMTSPIBuffer){ 0 };
}

static inline void apple_mt_spi_buf_ensure_capacity(AppleMTSPIBuffer *buf,
                                                    size_t bytes)
{
    if ((buf->len + bytes) > buf->capacity) {
        buf->capacity = buf->len + bytes;
        buf->data = g_realloc(buf->data, buf->capacity);
    }
}

static inline void apple_mt_spi_buf_set_capacity(AppleMTSPIBuffer *buf,
                                                 size_t capacity)
{
    assert_cmphex(capacity, >=, buf->capacity);
    buf->capacity = capacity;
    buf->data = g_realloc(buf->data, buf->capacity);
}

static inline void apple_mt_spi_buf_set_len(AppleMTSPIBuffer *buf, uint8_t val,
                                            size_t len)
{
    assert_cmphex(len, >=, buf->len);
    apple_mt_spi_buf_ensure_capacity(buf, len - buf->len);
    memset(buf->data + buf->len, val, len - buf->len);
    buf->len = len;
}

static inline bool apple_mt_spi_buf_is_empty(const AppleMTSPIBuffer *buf)
{
    return buf->len == 0;
}

static inline size_t apple_mt_spi_buf_get_pos(const AppleMTSPIBuffer *buf)
{
    assert_false(apple_mt_spi_buf_is_empty(buf));
    return buf->len - 1;
}

static inline bool apple_mt_spi_buf_is_full(const AppleMTSPIBuffer *buf)
{
    return !apple_mt_spi_buf_is_empty(buf) && buf->len == buf->capacity;
}

static inline bool apple_mt_spi_buf_pos_at_start(const AppleMTSPIBuffer *buf)
{
    return apple_mt_spi_buf_get_pos(buf) == 0;
}

static inline bool apple_mt_spi_buf_read_pos_at_end(const AppleMTSPIBuffer *buf)
{
    return (buf->read_pos + 1) == buf->len;
}

static inline void apple_mt_spi_buf_push_byte(AppleMTSPIBuffer *buf,
                                              uint8_t val)
{
    apple_mt_spi_buf_ensure_capacity(buf, sizeof(val));
    buf->data[buf->len] = val;
    buf->len += sizeof(val);
}

static inline void apple_mt_spi_buf_push_word(AppleMTSPIBuffer *buf,
                                              uint16_t val)
{
    apple_mt_spi_buf_ensure_capacity(buf, sizeof(val));
    stw_le_p(buf->data + buf->len, val);
    buf->len += sizeof(val);
}

static inline void apple_mt_spi_buf_push_dword(AppleMTSPIBuffer *buf,
                                               uint32_t val)
{
    apple_mt_spi_buf_ensure_capacity(buf, sizeof(val));
    stl_le_p(buf->data + buf->len, val);
    buf->len += sizeof(val);
}

static inline void apple_mt_spi_buf_push_data(AppleMTSPIBuffer *buf,
                                              const void *data, size_t len)
{
    if (len == 0) {
        return;
    }
    apple_mt_spi_buf_ensure_capacity(buf, len);
    memcpy(buf->data + buf->len, data, len);
    buf->len += len;
}

static inline void apple_mt_spi_buf_push_crc16(AppleMTSPIBuffer *buf)
{
    assert_false(apple_mt_spi_buf_is_empty(buf));
    apple_mt_spi_buf_push_word(buf, crc16(0, buf->data, buf->len));
}

static inline void apple_mt_spi_buf_append(AppleMTSPIBuffer *buf,
                                           AppleMTSPIBuffer *other_buf)
{
    apple_mt_spi_buf_push_data(buf, other_buf->data, other_buf->len);
    apple_mt_spi_buf_free(other_buf);
}

static inline uint8_t apple_mt_spi_buf_pop(AppleMTSPIBuffer *buf)
{
    uint8_t ret;

    if (apple_mt_spi_buf_is_empty(buf)) {
        return 0;
    }

    assert_nonnull(buf->data);
    assert_cmphex(buf->len, >, buf->read_pos);

    ret = buf->data[buf->read_pos];

    if (apple_mt_spi_buf_read_pos_at_end(buf) ||
        apple_mt_spi_buf_is_empty(buf)) {
        apple_mt_spi_buf_free(buf);
    } else {
        buf->read_pos += 1;
    }

    return ret;
}

static inline uint8_t apple_mt_spi_buf_read_byte(const AppleMTSPIBuffer *buf,
                                                 size_t off)
{
    assert_nonnull(buf->data);
    assert_cmphex(off, <, buf->len);
    return buf->data[off];
}

/// Reads a big-endian half word. HBPP puts its lengths on the wire that way
/// round; the link layer does not, and uses the packed structs above instead.
static inline uint16_t apple_mt_spi_buf_read_word_be(const AppleMTSPIBuffer *buf,
                                                     size_t off)
{
    assert_nonnull(buf->data);
    assert_cmphex(off + sizeof(uint16_t), <=, buf->len);
    return lduw_be_p(buf->data + off);
}

static void apple_mt_spi_push_pending_hbpp_word(AppleMTSPIState *s,
                                                uint16_t val)
{
    apple_mt_spi_buf_push_word(&s->pending_hbpp, val);
    qemu_irq_lower(s->irq);
}

static void apple_mt_spi_push_pending_hbpp_dword(AppleMTSPIState *s,
                                                 uint32_t val)
{
    apple_mt_spi_buf_push_dword(&s->pending_hbpp, val);
    qemu_irq_lower(s->irq);
}

static inline uint16_t apple_mt_spi_hbpp_packet_hdr_len(uint8_t val)
{
    switch (val) {
    case HBPP_PACKET_RESET:
        return 0x4;
    case HBPP_PACKET_NOP:
        return 0x2;
    case HBPP_PACKET_INT_ACK:
        return 0x2;
    case HBPP_PACKET_MEM_READ:
        return 0x8;
    case HBPP_PACKET_MEM_RMW:
        return 0x10;
    case HBPP_PACKET_REQ_CAL:
        return 0x2;
    case HBPP_PACKET_DATA:
        return 0xA;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: unknown HBPP packet type 0x%02X\n",
                      __func__, val);
        return 0x2;
    }
}

static void apple_mt_spi_handle_hbpp_data(AppleMTSPIState *s)
{
    uint32_t payload_len;
    uint32_t new_rx_capacity;

    if (!apple_mt_spi_buf_is_full(&s->rx)) {
        return;
    }

    // The length is a word count, so the guest can name a quarter of a
    // megabyte here. That is far more than the controller has room for, but
    // the arithmetic must not be allowed to wrap on the way to finding that
    // out, hence the 32-bit intermediates.
    payload_len = apple_mt_spi_buf_read_word_be(&s->rx, 2) * sizeof(uint32_t);
    new_rx_capacity = apple_mt_spi_hbpp_packet_hdr_len(HBPP_PACKET_DATA) +
                      payload_len + sizeof(uint32_t);

    if (s->rx.capacity == new_rx_capacity) {
        apple_mt_spi_push_pending_hbpp_word(s, HBPP_PACKET_ACK_DATA);
    } else if (new_rx_capacity > s->rx.capacity) {
        apple_mt_spi_buf_set_capacity(&s->rx, new_rx_capacity);
    } else {
        // The declared length shrank the packet below what has already been
        // clocked in. That cannot happen for a well-formed transfer, because
        // the field lives in the fixed header we have already consumed.
        // Dropping is the only safe answer: apple_mt_spi_buf_set_capacity()
        // only guards "buffers grow" with an assert_cmphex(), which this tree
        // compiles out with -DNDEBUG, so honouring it would g_realloc() the
        // buffer smaller while len still points past the new end and the next
        // byte pushed would land outside the allocation.
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: HBPP data length 0x%X would shrink the packet\n",
                      __func__, payload_len);
        apple_mt_spi_buf_free(&s->rx);
    }
}

static void apple_mt_spi_handle_hbpp_mem_rmw(AppleMTSPIState *s)
{
    if (apple_mt_spi_buf_is_full(&s->rx)) {
        apple_mt_spi_push_pending_hbpp_word(s, HBPP_PACKET_ACK_WR_REQ);
    }
}

static void apple_mt_spi_handle_hbpp(AppleMTSPIState *s)
{
    uint8_t packet_type;
    uint16_t hdr_len;

    packet_type = apple_mt_spi_buf_read_byte(&s->rx, 0);

    if (apple_mt_spi_buf_pos_at_start(&s->rx)) {
        hdr_len = apple_mt_spi_hbpp_packet_hdr_len(packet_type);
        // Same shrinking hazard as apple_mt_spi_handle_hbpp_data(): a header
        // shorter than what is already buffered would silently shrink the
        // allocation out from under buf->len rather than trip the compiled-out
        // assert. Not reachable for a well-formed packet, since every header
        // length is at least two bytes and only one has been clocked in.
        if (hdr_len < s->rx.capacity) {
            apple_mt_spi_buf_free(&s->rx);
            return;
        }
        apple_mt_spi_buf_set_capacity(&s->rx, hdr_len);
    }

    switch (packet_type) {
    case HBPP_PACKET_RESET:
        if (apple_mt_spi_buf_is_full(&s->rx)) {
            apple_mt_spi_push_pending_hbpp_word(s, HBPP_PACKET_REQ_BOOT);
        }
        break;
    case HBPP_PACKET_NOP:
        if (apple_mt_spi_buf_pos_at_start(&s->rx) &&
            apple_mt_spi_buf_is_empty(&s->tx)) {
            apple_mt_spi_buf_push_word(&s->tx, HBPP_PACKET_ACK_NOP);
        }
        break;
    case HBPP_PACKET_INT_ACK:
        if (apple_mt_spi_buf_pos_at_start(&s->rx)) {
            apple_mt_spi_buf_append(&s->tx, &s->pending_hbpp);
        }
        break;
    case HBPP_PACKET_MEM_READ:
        if (apple_mt_spi_buf_pos_at_start(&s->rx)) {
            apple_mt_spi_buf_ensure_capacity(&s->pending_hbpp, 2 + 4 + 2);
            apple_mt_spi_push_pending_hbpp_word(s, HBPP_PACKET_ACK_RD_REQ);
            apple_mt_spi_push_pending_hbpp_dword(s, 0x00000000); // value
            apple_mt_spi_push_pending_hbpp_word(s, 0x0000); // crc16 of value
        }
        break;
    case HBPP_PACKET_MEM_RMW:
        apple_mt_spi_handle_hbpp_mem_rmw(s);
        break;
    case HBPP_PACKET_REQ_CAL:
        if (apple_mt_spi_buf_pos_at_start(&s->rx)) {
            apple_mt_spi_push_pending_hbpp_word(s, HBPP_PACKET_CAL_DONE);
        }
        break;
    case HBPP_PACKET_DATA:
        apple_mt_spi_handle_hbpp_data(s);
        break;
    default:
        // apple_mt_spi_hbpp_packet_hdr_len() already complained about this one.
        break;
    }
}

static void apple_mt_spi_push_preamble(AppleMTSPIBuffer *buf)
{
    apple_mt_spi_buf_push_dword(buf, LL_PACKET_PREAMBLE);
}

static void apple_mt_spi_push_ll_hdr(AppleMTSPIBuffer *buf, uint8_t type,
                                     uint8_t interface, uint16_t payload_off,
                                     uint16_t payload_remaining,
                                     uint16_t payload_length)
{
    AppleMTSPILLHeader hdr;

    hdr.type = type;
    hdr.interface = interface;
    hdr.payload_off = cpu_to_le16(payload_off);
    hdr.payload_remaining = cpu_to_le16(payload_remaining);
    hdr.payload_length = cpu_to_le16(payload_length);

    apple_mt_spi_buf_push_data(buf, &hdr, sizeof(hdr));
}

static void apple_mt_spi_pad_ll_packet(AppleMTSPIBuffer *buf)
{
    apple_mt_spi_buf_set_len(
        buf, 0, LL_PACKET_LEN - sizeof(uint32_t) - sizeof(uint16_t));
}

static void apple_mt_spi_push_no_data(AppleMTSPIBuffer *buf)
{
    apple_mt_spi_push_ll_hdr(buf, LL_PACKET_NO_DATA, 0, 0, 0, 0);
    apple_mt_spi_pad_ll_packet(buf);
    apple_mt_spi_buf_push_crc16(buf);
}

/// Points @payload at the link-layer payload of the complete frame in @buf,
/// reports which interface it arrived on, and returns the payload length. That
/// length comes off the wire, so it is clamped to what the frame can actually
/// hold before anything indexes with it.
static uint16_t apple_mt_spi_ll_payload(const AppleMTSPIBuffer *buf,
                                        const uint8_t **payload,
                                        AppleMTSPILLHeader *hdr)
{
    if (buf->len < LL_PADDING_LEN + sizeof(*hdr)) {
        *payload = NULL;
        memset(hdr, 0, sizeof(*hdr));
        hdr->interface = LL_INTERFACE_HID;
        return 0;
    }

    memcpy(hdr, buf->data + LL_PADDING_LEN, sizeof(*hdr));
    hdr->payload_off = le16_to_cpu(hdr->payload_off);
    hdr->payload_remaining = le16_to_cpu(hdr->payload_remaining);
    hdr->payload_length = le16_to_cpu(hdr->payload_length);

    *payload = buf->data + LL_PADDING_LEN + sizeof(*hdr);
    // The declared length is guest-controlled. Clamp it to what a frame can
    // carry, which stops short of the trailing CRC.
    return MIN(hdr->payload_length,
               MIN(LL_PAYLOAD_MAX, buf->len - LL_PADDING_LEN - sizeof(*hdr)));
}

/// Reads the HID header out of a link-layer payload. Returns false, without
/// touching @hdr, if the payload is too short to hold one.
static bool apple_mt_spi_hid_hdr(const uint8_t *payload, uint16_t payload_len,
                                 AppleMTSPIHIDHeader *hdr)
{
    if (payload == NULL || payload_len < sizeof(*hdr)) {
        return false;
    }

    memcpy(hdr, payload, sizeof(*hdr));
    hdr->length_requested = le16_to_cpu(hdr->length_requested);
    hdr->payload_length = le16_to_cpu(hdr->payload_length);
    return true;
}

static void apple_mt_spi_push_hid_hdr(AppleMTSPIBuffer *buf, uint8_t type,
                                      uint8_t report_id, uint8_t packet_status,
                                      uint8_t frame_number,
                                      uint16_t length_requested,
                                      uint16_t payload_length)
{
    AppleMTSPIHIDHeader hdr;

    hdr.type = type;
    hdr.report_id = report_id;
    hdr.status = packet_status;
    hdr.frame_number = frame_number;
    hdr.length_requested = cpu_to_le16(length_requested);
    hdr.payload_length = cpu_to_le16(payload_length);

    apple_mt_spi_buf_push_data(buf, &hdr, sizeof(hdr));
}

/// Starts a HID report: the HID header, then the report ID byte that the host
/// counts as byte 0 of the report itself. @payload_length is the report's size
/// excluding that ID byte.
static void apple_mt_spi_push_report_hdr(AppleMTSPIBuffer *buf, uint8_t type,
                                         uint8_t report_id,
                                         uint8_t packet_status,
                                         uint8_t frame_number,
                                         uint16_t payload_length)
{
    apple_mt_spi_buf_ensure_capacity(buf, sizeof(AppleMTSPIHIDHeader) +
                                              sizeof(report_id) +
                                              payload_length);
    apple_mt_spi_push_hid_hdr(buf, type, report_id, packet_status, frame_number,
                              0, payload_length + sizeof(report_id));
    apple_mt_spi_buf_push_byte(buf, report_id);
}

static void apple_mt_spi_push_report_byte(AppleMTSPIBuffer *buf, uint8_t type,
                                          uint8_t report_id,
                                          uint8_t packet_status,
                                          uint8_t frame_number, uint8_t val)
{
    apple_mt_spi_push_report_hdr(buf, type, report_id, packet_status,
                                 frame_number, sizeof(val));
    apple_mt_spi_buf_push_byte(buf, val);
}

static AppleMTSPILLPacket *apple_mt_spi_new_packet(uint8_t type,
                                                   uint8_t interface)
{
    AppleMTSPILLPacket *packet;

    packet = g_new0(AppleMTSPILLPacket, 1);
    packet->type = type;
    packet->interface = interface;
    return packet;
}

/// Seals a packet with its CRC, queues it for the next transaction and lets
/// the host know there is something to collect.
static void apple_mt_spi_queue_packet(AppleMTSPIState *s,
                                      AppleMTSPILLPacket *packet)
{
    apple_mt_spi_buf_push_crc16(&packet->buf);

    /*
     * Nothing here fragments, so a payload that does not fit is a bug in this
     * file rather than something the guest can provoke - but it has to be a
     * real check, not an assertion. QEMU is built with NDEBUG, which turns the
     * assert_*() family into no-ops, and the failure mode downstream is not a
     * clean abort: apple_mt_spi_pad_ll_packet() would compute a negative
     * padding length, underflow it through size_t and memset() a wild length.
     * The 449-byte transport statistics report leaves only ~40 bytes of
     * headroom, so keep the guard honest.
     */
    if (packet->buf.len > LL_PAYLOAD_MAX) {
        error_report("%s: dropping an oversized %u byte packet of type 0x%02X",
                     __func__, packet->buf.len, packet->type);
        apple_mt_spi_buf_free(&packet->buf);
        g_free(packet);
        return;
    }

    QTAILQ_INSERT_TAIL(&s->pending_fw, packet, next);
    qemu_irq_lower(s->irq);
}

/// Builds HID report 0xF8, the transport statistics documented above. Counters
/// this controller has nothing truthful to say about stay zero, which is what
/// a controller that never hit the condition would report.
static void apple_mt_spi_push_transport_stats(AppleMTSPIState *s,
                                              AppleMTSPIBuffer *buf,
                                              uint8_t frame_number)
{
    // Sized without the leading report ID byte, which push_report_hdr() emits,
    // hence the -1 on every offset below.
    uint8_t report[MT_TRANSPORT_STATS_LEN - 1] = { 0 };

    stl_le_p(report + MT_STATS_OFF_GET_REPORT_COUNT - 1, s->stats.get_reports);
    stl_le_p(report + MT_STATS_OFF_SET_REPORT_COUNT - 1, s->stats.set_reports);
    stl_le_p(report + MT_STATS_OFF_INPUT_REPORT_COUNT - 1,
             s->stats.input_reports);
    stl_le_p(report + MT_STATS_OFF_INPUT_DROP_COUNT - 1, s->stats.input_drops);
    stl_le_p(report + MT_STATS_OFF_OUTPUT_REPORT_COUNT - 1,
             s->stats.output_reports);
    stl_le_p(report + MT_STATS_OFF_NO_PIPELINE_ITEM - 1, 0);

    apple_mt_spi_push_report_hdr(buf, HID_CONTROL_PACKET_SET_OUTPUT_REPORT,
                                 HID_REPORT_TRANSPORT_STATS,
                                 HID_PACKET_STATUS_SUCCESS, frame_number,
                                 sizeof(report));
    apple_mt_spi_buf_push_data(buf, report, sizeof(report));
}

/// Builds HID report 0x72, the power state residency documented above. The
/// emulated panel never leaves its scanning state, so the whole interval since
/// the counters were last cleared is charged to "Active".
static void apple_mt_spi_push_power_stats(AppleMTSPIState *s,
                                          AppleMTSPIBuffer *buf,
                                          uint8_t frame_number)
{
    uint8_t report[MT_POWER_STATS_LEN - 1] = { 0 };
    uint64_t elapsed_ns;
    uint64_t ticks;
    size_t off;

    elapsed_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->power_stats_since;
    ticks = elapsed_ns /
            (NANOSECONDS_PER_SECOND / MT_POWER_STATS_TICKS_PER_SECOND);
    ticks = MIN(ticks, UINT32_MAX);

    off = MT_POWER_STATS_OFF_HIGH_LEVEL - 1 +
          MT_POWER_STATE_ACTIVE * MT_POWER_STATS_RECORD_LEN;
    stl_le_p(report + off, (uint32_t)ticks);
    // One entry into the state, at power-on. A state with residency but no
    // entries would read as a controller contradicting itself.
    stl_le_p(report + off + sizeof(uint32_t), 1);

    apple_mt_spi_push_report_hdr(buf, HID_CONTROL_PACKET_SET_OUTPUT_REPORT,
                                 HID_REPORT_POWER_STATS,
                                 HID_PACKET_STATUS_SUCCESS, frame_number,
                                 sizeof(report));
    apple_mt_spi_buf_push_data(buf, report, sizeof(report));
}

static void apple_mt_spi_handle_get_feature(AppleMTSPIState *s,
                                            const AppleMTSPIHIDHeader *req,
                                            uint8_t interface)
{
    AppleMTSPILLPacket *packet;
    uint8_t report_id;
    uint8_t frame_number;

    report_id = req->report_id;
    frame_number = req->frame_number;

    s->stats.get_reports++;

    packet = apple_mt_spi_new_packet(LL_PACKET_CONTROL, interface);

    switch (report_id) {
    case HID_REPORT_FAMILY_ID:
        apple_mt_spi_buf_ensure_capacity(&packet->buf, 9 + 1 + 2);
        apple_mt_spi_push_report_byte(
            &packet->buf, HID_CONTROL_PACKET_SET_OUTPUT_REPORT, report_id,
            HID_PACKET_STATUS_SUCCESS, frame_number, MT_FAMILY_ID);
        break;
    case HID_REPORT_BASIC_DEVICE_INFO:
        apple_mt_spi_buf_ensure_capacity(&packet->buf, 9 + 5 + 2);
        apple_mt_spi_push_report_hdr(
            &packet->buf, HID_CONTROL_PACKET_SET_OUTPUT_REPORT, report_id,
            HID_PACKET_STATUS_SUCCESS, frame_number, 5);
        apple_mt_spi_buf_push_byte(&packet->buf, MT_LITTLE_ENDIAN);
        apple_mt_spi_buf_push_byte(&packet->buf, MT_ROWS);
        apple_mt_spi_buf_push_byte(&packet->buf, MT_COLUMNS);
        apple_mt_spi_buf_push_word(&packet->buf, MT_BCD_VER);
        break;
    case HID_REPORT_SENSOR_SURFACE_DESC:
        apple_mt_spi_buf_ensure_capacity(&packet->buf, 9 + 16 + 2);
        apple_mt_spi_push_report_hdr(
            &packet->buf, HID_CONTROL_PACKET_SET_OUTPUT_REPORT, report_id,
            HID_PACKET_STATUS_SUCCESS, frame_number, 16);
        apple_mt_spi_buf_push_dword(&packet->buf, MT_SENSOR_SURFACE_WIDTH);
        apple_mt_spi_buf_push_dword(&packet->buf, MT_SENSOR_SURFACE_HEIGHT);
        // these values might need to be different, especially considering the
        // values/stuff inside HID_REPORT_SENSOR_REGION_DESC.
        apple_mt_spi_buf_push_word(&packet->buf, 0);
        apple_mt_spi_buf_push_word(&packet->buf, 0);
        apple_mt_spi_buf_push_word(&packet->buf, MT_SENSOR_SURFACE_WIDTH);
        apple_mt_spi_buf_push_word(&packet->buf, MT_SENSOR_SURFACE_HEIGHT);
        break;
    case HID_REPORT_SENSOR_REGION_PARAM:
        apple_mt_spi_buf_ensure_capacity(&packet->buf, 9 + 6 + 2);
        apple_mt_spi_push_report_hdr(
            &packet->buf, HID_CONTROL_PACKET_SET_OUTPUT_REPORT, report_id,
            HID_PACKET_STATUS_SUCCESS, frame_number, 6);
        apple_mt_spi_buf_push_word(&packet->buf, 0x0);
        apple_mt_spi_buf_push_word(&packet->buf, 0x7);
        apple_mt_spi_buf_push_word(&packet->buf, 0x200);
        break;
    case HID_REPORT_SENSOR_REGION_DESC:
        apple_mt_spi_buf_ensure_capacity(&packet->buf, 9 + 22 + 2);
        apple_mt_spi_push_report_hdr(
            &packet->buf, HID_CONTROL_PACKET_SET_OUTPUT_REPORT, report_id,
            HID_PACKET_STATUS_SUCCESS, frame_number, 22); // 1 + 7*3
        apple_mt_spi_buf_push_byte(&packet->buf, 3); // region count

        apple_mt_spi_buf_push_byte(&packet->buf, 1); // type Multitouch
        apple_mt_spi_buf_push_byte(&packet->buf, 0); // start_row
        apple_mt_spi_buf_push_byte(&packet->buf, 30); // rows
        apple_mt_spi_buf_push_byte(&packet->buf, 1); // row_skip
        apple_mt_spi_buf_push_byte(&packet->buf, 0); // start_col
        apple_mt_spi_buf_push_byte(&packet->buf, 14); // cols
        apple_mt_spi_buf_push_byte(&packet->buf, 0); // hardware_coloffset

        apple_mt_spi_buf_push_byte(&packet->buf, 8); // type CommonMode
        apple_mt_spi_buf_push_byte(&packet->buf, 0); // start_row
        apple_mt_spi_buf_push_byte(&packet->buf, 30); // rows
        apple_mt_spi_buf_push_byte(&packet->buf, 1); // row_skip
        apple_mt_spi_buf_push_byte(&packet->buf, 14); // start_col
        apple_mt_spi_buf_push_byte(&packet->buf, 1); // cols
        apple_mt_spi_buf_push_byte(&packet->buf, 0); // hardware_coloffset

        apple_mt_spi_buf_push_byte(&packet->buf, 11); // type Unknown
        apple_mt_spi_buf_push_byte(&packet->buf, 30); // maybe rows?
        apple_mt_spi_buf_push_byte(&packet->buf, 1); // maybe row_skip?
        apple_mt_spi_buf_push_byte(&packet->buf, 1); // unknown
        apple_mt_spi_buf_push_byte(&packet->buf, 0); // unknown
        // maybe ignored? offsets 0x14/0x15 > size 0x14
        apple_mt_spi_buf_push_byte(&packet->buf, 15);
        apple_mt_spi_buf_push_byte(&packet->buf, 0);
        break;
    case HID_REPORT_STATUS:
        /*
         * The controller's latched critical errors. AppleMultitouchDevice::
         * decodeDeviceProperty() accepts a payload of exactly two or exactly
         * four bytes and reads it little-endian; any other length silently
         * decodes as zero. Nothing has gone wrong with an emulated controller,
         * so answer the honest zero at a length it actually parses.
         */
        apple_mt_spi_push_report_hdr(
            &packet->buf, HID_CONTROL_PACKET_SET_OUTPUT_REPORT, report_id,
            HID_PACKET_STATUS_SUCCESS, frame_number, MT_STATUS_LEN);
        apple_mt_spi_buf_push_dword(&packet->buf, 0);
        break;
    case HID_REPORT_TRANSPORT_STATS:
        apple_mt_spi_push_transport_stats(s, &packet->buf, frame_number);
        break;
    case HID_REPORT_POWER_STATS:
        apple_mt_spi_push_power_stats(s, &packet->buf, frame_number);
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "%s: unimplemented feature report 0x%02X, answering "
                      "with a zero byte\n",
                      __func__, report_id);
        apple_mt_spi_push_report_byte(
            &packet->buf, HID_CONTROL_PACKET_SET_OUTPUT_REPORT, report_id,
            HID_PACKET_STATUS_SUCCESS, frame_number, 0);
        break;
    }

    apple_mt_spi_queue_packet(s, packet);
}

/// Applies a report the host has written to the controller. Only the two
/// statistics reports have state behind them, and for those a write is the
/// host clearing the interval it has just read.
static void apple_mt_spi_apply_set_report(AppleMTSPIState *s, uint8_t report_id)
{
    switch (report_id) {
    case HID_REPORT_TRANSPORT_STATS:
        s->stats = (AppleMTSPIStats){ 0 };
        break;
    case HID_REPORT_POWER_STATS:
        s->power_stats_since = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "%s: ignoring write of report 0x%02X\n",
                      __func__, report_id);
        break;
    }
}

static void apple_mt_spi_handle_set_feature(AppleMTSPIState *s,
                                            const AppleMTSPIHIDHeader *req,
                                            uint8_t interface)
{
    AppleMTSPILLPacket *packet;

    s->stats.set_reports++;
    apple_mt_spi_apply_set_report(s, req->report_id);

    packet = apple_mt_spi_new_packet(LL_PACKET_CONTROL, interface);
    apple_mt_spi_push_hid_hdr(&packet->buf,
                              HID_CONTROL_PACKET_SET_OUTPUT_REPORT,
                              req->report_id, HID_PACKET_STATUS_SUCCESS,
                              req->frame_number, 0, 0);
    apple_mt_spi_queue_packet(s, packet);
}

static void apple_mt_spi_handle_control(AppleMTSPIState *s)
{
    AppleMTSPIHIDHeader req;
    AppleMTSPILLHeader hdr;
    const uint8_t *payload;
    uint16_t payload_len;

    payload_len = apple_mt_spi_ll_payload(&s->rx, &payload, &hdr);

    if (!apple_mt_spi_hid_hdr(payload, payload_len, &req)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: control packet with a %u byte payload is too short "
                      "for a HID header\n",
                      __func__, payload_len);
        return;
    }

    switch (req.type) {
    case HID_CONTROL_PACKET_GET_FEATURE_REPORT:
        apple_mt_spi_handle_get_feature(s, &req, hdr.interface);
        break;
    case HID_CONTROL_PACKET_SET_FEATURE_REPORT:
        apple_mt_spi_handle_set_feature(s, &req, hdr.interface);
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "%s: unimplemented HID control packet type 0x%02X for "
                      "report 0x%02X\n",
                      __func__, req.type, req.report_id);
        break;
    }
}

/*
 * Handles a data frame the host sent us.
 *
 * Two different things arrive this way. Frames on the device interface are
 * addressed to the controller itself and carry no HID header; see below. On a
 * HID interface these are HID output reports: AppleHIDTransportProtocolHIDSPI
 * ::controlReportGated() picks the HID packet type from the report type and
 * direction, and for a *set* of an Output report it selects 0x20 - the data
 * transfer type - rather than 0x51, sending it as a data frame instead of a
 * control packet. Feature reports keep going through the control path, so this
 * is not where the statistics reports get cleared.
 */
static void apple_mt_spi_handle_output(AppleMTSPIState *s)
{
    AppleMTSPIHIDHeader req;
    AppleMTSPILLHeader hdr;
    const uint8_t *payload;
    uint16_t payload_len;

    payload_len = apple_mt_spi_ll_payload(&s->rx, &payload, &hdr);

    if (hdr.interface == LL_INTERFACE_DEVICE) {
        /*
         * Addressed to the controller itself, not to its HID interface, so
         * there is no HID header here and none is expected. iOS sends exactly
         * two of these per boot, both four bytes:
         *
         *     A0 10 02 00     and     A0 10 12 00
         *
         * It asks for no reply and does not wait for one: enumeration, the
         * feature reports and touch all complete without the controller ever
         * answering. The encoding is not understood beyond that, so say so
         * with the bytes attached rather than guessing at a response.
         */
        qemu_log_mask(LOG_UNIMP,
                      "%s: unimplemented %u byte device-interface request "
                      "%02X %02X %02X %02X\n",
                      __func__, payload_len, payload_len > 0 ? payload[0] : 0,
                      payload_len > 1 ? payload[1] : 0,
                      payload_len > 2 ? payload[2] : 0,
                      payload_len > 3 ? payload[3] : 0);
        return;
    }

    if (!apple_mt_spi_hid_hdr(payload, payload_len, &req)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: output frame on interface 0x%02X with a %u byte "
                      "payload is too short for a HID header\n",
                      __func__, hdr.interface, payload_len);
        s->stats.input_drops++;
        return;
    }

    if (req.type != HID_TRANSFER_PACKET_OUTPUT) {
        qemu_log_mask(LOG_UNIMP,
                      "%s: unimplemented HID transfer type 0x%02X for report "
                      "0x%02X\n",
                      __func__, req.type, req.report_id);
    }

    s->stats.output_reports++;
    apple_mt_spi_apply_set_report(s, req.report_id);

    // Nothing goes back. The host does not wait on these: it clocks the frame
    // out and carries on. The only thing a lossless transfer adds is that the
    // host would retry one the controller had reported itself busy for, and
    // this controller is never busy.
}

static void apple_mt_spi_handle_fw_packet(AppleMTSPIState *s)
{
    uint8_t packet_type;
    AppleMTSPIBuffer buf = { 0 };
    AppleMTSPILLPacket *packet;

    if (apple_mt_spi_buf_get_pos(&s->rx) == sizeof(uint32_t)) {
        apple_mt_spi_buf_set_capacity(&s->rx, LL_PACKET_LEN);

        if (QTAILQ_EMPTY(&s->pending_fw)) {
            apple_mt_spi_push_no_data(&buf);
        } else {
            packet = QTAILQ_FIRST(&s->pending_fw);
            assert_nonnull(packet);
            apple_mt_spi_push_ll_hdr(&buf, packet->type, packet->interface, 0,
                                     0, packet->buf.len);
            apple_mt_spi_buf_append(&buf, &packet->buf);
            apple_mt_spi_pad_ll_packet(&buf);
            apple_mt_spi_buf_push_crc16(&buf);
            QTAILQ_REMOVE(&s->pending_fw, packet, next);
            g_free(packet);
            packet = NULL;
        }

        apple_mt_spi_buf_append(&s->tx, &buf);
    }

    if (!apple_mt_spi_buf_is_full(&s->rx)) {
        return;
    }

    packet_type = apple_mt_spi_buf_read_byte(&s->rx, sizeof(uint32_t));

    switch (packet_type) {
    case LL_PACKET_NO_DATA:
        break;
    case LL_PACKET_CONTROL:
        apple_mt_spi_handle_control(s);
        break;
    case LL_PACKET_LOSSLESS_OUTPUT:
    case LL_PACKET_LOSSY_OUTPUT:
        apple_mt_spi_handle_output(s);
        break;
    case LL_PACKET_LOSSLESS_INPUT:
    case LL_PACKET_LOSSY_INPUT:
        // These are the types the host would use to hand a HID input report
        // down to a device that has one. iOS has not been observed sending
        // them to this controller. Count it as a drop, so the transport
        // statistics say so rather than pretending it was consumed.
        s->stats.input_drops++;
        qemu_log_mask(LOG_UNIMP, "%s: unimplemented LL packet type 0x%02X\n",
                      __func__, packet_type);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: unknown LL packet type 0x%02X\n",
                      __func__, packet_type);
        break;
    }
}

static void apple_mt_spi_handle_fw(AppleMTSPIState *s)
{
    uint8_t packet_type;

    if (apple_mt_spi_buf_pos_at_start(&s->rx)) {
        apple_mt_spi_buf_set_capacity(&s->rx, sizeof(uint32_t) * 2);
        apple_mt_spi_push_preamble(&s->tx);
    }

    if (apple_mt_spi_buf_get_pos(&s->rx) < sizeof(uint32_t)) {
        return;
    }

    packet_type = apple_mt_spi_buf_read_byte(&s->rx, sizeof(uint32_t));

    switch (packet_type) {
    case LL_PACKET_ERROR & 0xFF:
    case LL_PACKET_ACK & 0xFF:
    case LL_PACKET_NAK & 0xFF:
    case LL_PACKET_BUSY & 0xFF:
        if (apple_mt_spi_buf_get_pos(&s->rx) == sizeof(uint32_t)) {
            apple_mt_spi_buf_push_dword(&s->tx, LL_PACKET_ACK);
        }
        break;
    default:
        apple_mt_spi_handle_fw_packet(s);
        break;
    }
}

static uint32_t apple_mt_spi_transfer(SSIPeripheral *dev, uint32_t val)
{
    AppleMTSPIState *s;
    uint8_t ret;

    s = container_of(dev, AppleMTSPIState, parent_obj);

    QEMU_LOCK_GUARD(&s->lock);

    apple_mt_spi_buf_push_byte(&s->rx, (uint8_t)val);

    if (apple_mt_spi_buf_read_byte(&s->rx, 0) == (LL_PACKET_PREAMBLE & 0xFF)) {
        apple_mt_spi_handle_fw(s);
    } else {
        apple_mt_spi_handle_hbpp(s);
    }

    if (apple_mt_spi_buf_is_full(&s->rx)) {
        apple_mt_spi_buf_free(&s->rx);
    }

    ret = apple_mt_spi_buf_pop(&s->tx);

    if (apple_mt_spi_buf_is_empty(&s->pending_hbpp) &&
        QTAILQ_EMPTY(&s->pending_fw)) {
        qemu_irq_raise(s->irq);
    } else {
        qemu_irq_lower(s->irq);
    }

    return ret;
}

static void apple_mt_spi_send_path_update(AppleMTSPIState *s, uint64_t ts,
                                          uint8_t path_stage)
{
    AppleMTSPILLPacket *packet;
    AppleMTSPIFrameHeader frame;
    AppleMTSPIPath path;
    uint64_t ts_delta_ms;
    int32_t x_delta;
    int32_t y_delta;

    ts_delta_ms = (ts - s->prev_ts) / SCALE_MS;
    ts_delta_ms = MAX(ts_delta_ms, 1); // Prevent div-by-zero
    s->prev_ts = ts;

    x_delta = s->x - s->prev_x;
    y_delta = s->y - s->prev_y;

    memset(&frame, 0, sizeof(frame));
    frame.frame_number = s->frame;
    frame.header_len = sizeof(frame) + sizeof(uint8_t /* report ID */);
    frame.timestamp = cpu_to_le32(ts / SCALE_MS);
    frame.image_len = cpu_to_le16(0);
    frame.path_count = 1;
    frame.path_len = sizeof(path);

    memset(&path, 0, sizeof(path));
    path.id = 1;
    path.stage = path_stage;
    path.finger_id = 1;
    path.hand_id = 1;
    path.x = cpu_to_le16(s->x);
    path.y = cpu_to_le16(s->y);
    // Surface units per second. The previous form divided by a nanosecond
    // delta before scaling, so it truncated to zero for every plausible
    // movement; dividing by the millisecond delta is what was meant. Both
    // components are magnitudes, as the sign is carried by the position
    // deltas iOS computes itself, and both are clamped because a fast swipe
    // between two consecutive samples otherwise overflows the 16-bit field.
    path.vel_x = cpu_to_le16(MIN(ABS(x_delta) * 1000 / ts_delta_ms, INT16_MAX));
    path.vel_y = cpu_to_le16(MIN(ABS(y_delta) * 1000 / ts_delta_ms, INT16_MAX));
    // A fingertip is roughly 6.6 mm by 5.8 mm in the surface units of
    // MT_SENSOR_SURFACE_WIDTH. iOS turns these into a contact density itself,
    // as contactDensityByRadii = (scale * 400) / (sqrt(major * minor) - min),
    // so there is nothing to compute here beyond a plausible ellipse.
    path.radius_major = cpu_to_le16(660);
    path.radius_minor = cpu_to_le16(580);
    path.orientation = cpu_to_le16(19317);
    path.radius_scale = cpu_to_le16(100);

    packet = apple_mt_spi_new_packet(LL_PACKET_LOSSLESS_OUTPUT,
                                     LL_INTERFACE_HID);
    apple_mt_spi_push_report_hdr(&packet->buf, HID_TRANSFER_PACKET_OUTPUT,
                                 HID_REPORT_BINARY_PATH_OR_IMAGE,
                                 HID_PACKET_STATUS_SUCCESS, s->frame,
                                 sizeof(frame) + sizeof(path));
    apple_mt_spi_buf_push_data(&packet->buf, &frame, sizeof(frame));
    apple_mt_spi_buf_push_data(&packet->buf, &path, sizeof(path));

    s->frame++;
    s->stats.input_reports++;

    apple_mt_spi_queue_packet(s, packet);
}

typedef struct {
    AppleMTSPIState *s;
    uint64_t ts;
    uint8_t path_stage;
} AppleMTSPITouchUpdate;

static AppleMTSPITouchUpdate *apple_mt_spi_new_touch_update(AppleMTSPIState *s,
                                                            uint8_t path_stage)
{
    AppleMTSPITouchUpdate *update = g_new(AppleMTSPITouchUpdate, 1);
    update->s = s;
    update->ts = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    update->path_stage = path_stage;
    return update;
}

static void apple_mt_spi_send_touch_update_bh(void *opaque)
{
    AppleMTSPITouchUpdate *update = opaque;

    QEMU_LOCK_GUARD(&update->s->lock);

    apple_mt_spi_send_path_update(update->s, update->ts, update->path_stage);

    g_free(opaque);
}

static void apple_mt_spi_schedule_touch_update(AppleMTSPIState *s,
                                               uint8_t path_stage)
{
    aio_bh_schedule_oneshot(qemu_get_aio_context(),
                            apple_mt_spi_send_touch_update_bh,
                            apple_mt_spi_new_touch_update(s, path_stage));
}

static void apple_mt_spi_timer_tick(void *opaque)
{
    AppleMTSPIState *s = opaque;

    QEMU_LOCK_GUARD(&s->lock);

    if (s->prev_x != s->x || s->prev_y != s->y) {
        apple_mt_spi_schedule_touch_update(s, PATH_STAGE_TOUCHING);
    }

    if (s->btn_state & MOUSE_EVENT_LBUTTON) {
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                NANOSECONDS_PER_SECOND / 20);
    }
}

static void apple_mt_spi_end_timer_tick(void *opaque)
{
    AppleMTSPIState *s = opaque;

    QEMU_LOCK_GUARD(&s->lock);

    apple_mt_spi_schedule_touch_update(s, PATH_STAGE_OUT_OF_RANGE);

    s->prev_ts = 0;
    s->prev_x = 0;
    s->prev_y = 0;
}

static void apple_mt_spi_mouse_event(void *opaque, int dx, int dy, int dz,
                                     int buttons_state)
{
    AppleMTSPIState *s = opaque;

    QEMU_LOCK_GUARD(&s->lock);

    s->prev_x = s->x;
    s->prev_y = s->y;
    s->x = qemu_input_scale_axis(dx, INPUT_EVENT_ABS_MIN, INPUT_EVENT_ABS_MAX,
                                 0, MT_SENSOR_SURFACE_WIDTH);
    s->y =
        qemu_input_scale_axis(INPUT_EVENT_ABS_MAX - dy, INPUT_EVENT_ABS_MIN,
                              INPUT_EVENT_ABS_MAX, 0, MT_SENSOR_SURFACE_HEIGHT);
    // Hardcoded calibration on y-axis.
    // Tested accuracy for display_height 1792 is +/- 1 pixel.
    // it might not be perfect, also there might be some calibration needed for
    // "x".
    // s->y -= qemu_input_scale_axis(16, 0, 1792, 0,
    // MT_SENSOR_SURFACE_HEIGHT);
    // fprintf(stderr, "%s: display_height: %u ; display_width: %u\n", __func__,
    //         s->display_height, s->display_width);
    s->y -= qemu_input_scale_axis(16, 0, s->display_height, 0,
                                  MT_SENSOR_SURFACE_HEIGHT);
    s->prev_btn_state = s->btn_state;
    s->btn_state = buttons_state;

    if ((s->prev_btn_state & MOUSE_EVENT_LBUTTON) == 0 &&
        (s->btn_state & MOUSE_EVENT_LBUTTON) != 0) {
        apple_mt_spi_schedule_touch_update(s, PATH_STAGE_MAKE_TOUCH);

        timer_del(s->end_timer);
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                NANOSECONDS_PER_SECOND / 10);
    } else if ((s->prev_btn_state & MOUSE_EVENT_LBUTTON) != 0 &&
               (s->btn_state & MOUSE_EVENT_LBUTTON) == 0) {
        apple_mt_spi_schedule_touch_update(s, PATH_STAGE_BREAK_TOUCH);

        timer_del(s->timer);
        timer_mod(s->end_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                    NANOSECONDS_PER_SECOND / 10);
    }
}
static void apple_mt_spi_realize(SSIPeripheral *dev, Error **errp)
{
    AppleMTSPIState *s;
    QEMUPutMouseEntry *entry;

    s = container_of(dev, AppleMTSPIState, parent_obj);

    entry = qemu_add_mouse_event_handler(apple_mt_spi_mouse_event, s, 1,
                                         "Apple Multitouch HID SPI");
    qemu_activate_mouse_event_handler(entry);
}

static const Property apple_mt_spi_props[] = {
    DEFINE_PROP_UINT32("display_width", AppleMTSPIState, display_width, 0),
    DEFINE_PROP_UINT32("display_height", AppleMTSPIState, display_height, 0),
};

static void apple_mt_spi_reset_enter(Object *obj, ResetType type)
{
    AppleMTSPIState *s;
    AppleMTSPILLPacket *packet;
    AppleMTSPILLPacket *packet_next;

    s = APPLE_MT_SPI(obj);

    QEMU_LOCK_GUARD(&s->lock);

    timer_del(s->timer);
    timer_del(s->end_timer);

    s->btn_state = 0;
    s->prev_btn_state = 0;
    s->prev_x = 0;
    s->prev_y = 0;
    s->x = 0;
    s->y = 0;
    s->prev_ts = 0;
    s->frame = 0;
    s->stats = (AppleMTSPIStats){ 0 };
    s->power_stats_since = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    apple_mt_spi_buf_free(&s->tx);
    apple_mt_spi_buf_free(&s->rx);
    apple_mt_spi_buf_free(&s->pending_hbpp);

    QTAILQ_FOREACH_SAFE (packet, &s->pending_fw, next, packet_next) {
        QTAILQ_REMOVE(&s->pending_fw, packet, next);
        apple_mt_spi_buf_free(&packet->buf);
        g_free(packet);
    }
}

static void apple_mt_spi_reset_hold(Object *obj, ResetType type)
{
    AppleMTSPIState *s;

    s = APPLE_MT_SPI(obj);

    QEMU_LOCK_GUARD(&s->lock);

    qemu_irq_raise(s->irq);
}

static void apple_mt_spi_class_init(ObjectClass *klass, const void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);

    rc->phases.enter = apple_mt_spi_reset_enter;
    rc->phases.hold = apple_mt_spi_reset_hold;

    dc->user_creatable = false;
    dc->vmsd = &vmstate_apple_mt_spi;
    device_class_set_props(dc, apple_mt_spi_props);
    set_bit(DEVICE_CATEGORY_INPUT, dc->categories);

    k->realize = apple_mt_spi_realize;
    k->transfer = apple_mt_spi_transfer;
}

static void apple_mt_instance_init(Object *obj)
{
    AppleMTSPIState *s;

    s = APPLE_MT_SPI(obj);

    // These layouts are wire formats; padding one of them would silently
    // corrupt every report the controller sends.
    QEMU_BUILD_BUG_ON(sizeof(AppleMTSPILLHeader) != 8);
    QEMU_BUILD_BUG_ON(sizeof(AppleMTSPIHIDHeader) != 8);
    QEMU_BUILD_BUG_ON(sizeof(AppleMTSPIFrameHeader) != 27);
    QEMU_BUILD_BUG_ON(sizeof(AppleMTSPIPath) != 20);

    qdev_init_gpio_out_named(DEVICE(s), &s->irq, APPLE_MT_SPI_IRQ, 1);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, apple_mt_spi_timer_tick, s);
    s->end_timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL, apple_mt_spi_end_timer_tick, s);

    QTAILQ_INIT(&s->pending_fw);

    qemu_mutex_init(&s->lock);
}

static const TypeInfo apple_mt_spi_type_info = {
    .name = TYPE_APPLE_MT_SPI,
    .parent = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(AppleMTSPIState),
    .instance_init = apple_mt_instance_init,
    .class_init = apple_mt_spi_class_init,
};

static void apple_mt_spi_register_types(void)
{
    type_register_static(&apple_mt_spi_type_info);
}

type_init(apple_mt_spi_register_types);
