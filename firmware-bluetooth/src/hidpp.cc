/*
 * hidpp.cc — HID++ 2.0 ReprogControls diversion for hid-remapper BT firmware
 *
 * See hidpp.h for the full design description.
 *
 * Threading model
 * ---------------
 * All functions that touch per-connection state are called from the system
 * workqueue (hogp_ready_work, the main loop, and BT connection callbacks
 * dispatched via k_work).  The only exception is hidpp_handle_input_report()
 * which is called from the main() loop — but that loop is single-threaded for
 * report processing, so no additional locking is needed here.
 *
 * GATT write notes
 * ----------------
 * BLE HID Report characteristics carry the payload WITHOUT the report-ID byte.
 * The report ID is identified by the Report Reference descriptor (UUID 0x2908)
 * attached to the characteristic.  Accordingly, when we write a HID++ short
 * or long report we strip the leading 0x10/0x11 report-ID byte and write only
 * the 6 or 19 payload bytes.
 *
 * Conversely, INPUT notifications also arrive without the report-ID byte.
 * main.cc's hogp_notify_cb() prefixes bt_hogp_rep_id() as data[0] and stores
 * the full [reportId, payload...] in report_q.  hidpp_handle_input_report()
 * receives (report_id, payload, len) already split that way.
 */

#include "hidpp.h"

#include <string.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(hidpp, LOG_LEVEL_DBG);

/* -------------------------------------------------------------------------
 * Internal state
 * ---------------------------------------------------------------------- */

enum class HidppState : uint8_t {
    IDLE,               /* No HID++ output characteristic found */
    PROBING,            /* Sent GetFeature(0x1b04); waiting for response */
    DIVERTING,          /* Sent SetControlReporting; waiting for ack */
    ACTIVE,             /* Diversion live; listening for events */
};

struct HidppConn {
    HidppState  state;
    uint16_t    out_handle;   /* GATT value handle for the single OUTPUT Report
                               * characteristic (0x0030 on the Lift).  Both
                               * short (0x10) and long (0x11) HID++ writes go
                               * here — the Lift exposes only one writable
                               * Report characteristic, distinguished from the
                               * two INPUT (NOTIFY) ones by the absence of a
                               * CCCD (0x2902) descriptor. */
    uint8_t     feature_idx;  /* 0x1b04 feature index (set after PROBING) */
    bool        dpi_pressed;  /* Last known state of DPI button */
    bool        descriptor_patched; /* True once amend_descriptor() has run */
};

static HidppConn s_conns[HIDPP_MAX_CONN];

/* -------------------------------------------------------------------------
 * Helpers — GATT UUIDs (Bluetooth SIG)
 * ---------------------------------------------------------------------- */

/* HID Report characteristic: 0x2a4d */
static const struct bt_uuid_16 UUID_REPORT =
    BT_UUID_INIT_16(0x2a4d);

/* Report Reference descriptor: 0x2908 */
static const struct bt_uuid_16 UUID_REPORT_REF =
    BT_UUID_INIT_16(0x2908);

/* Client Characteristic Configuration descriptor: 0x2902
 * Presence of this descriptor on a Report char means INPUT (NOTIFY-capable).
 * Absence means OUTPUT or FEATURE (write-only) — the one we want. */
static const struct bt_uuid_16 UUID_CCCD =
    BT_UUID_INIT_16(0x2902);

/* -------------------------------------------------------------------------
 * Helpers — HID++ packet construction
 * ---------------------------------------------------------------------- */

/*
 * Build a HID++ 2.0 long report payload (19 bytes written to the OUTPUT
 * Report characteristic, without the report-ID prefix byte).
 *
 * The Lift over BLE uses only long (0x11) reports for HID++ in both
 * directions.  There is no 0x10 Output characteristic.
 *
 *  out19[0] = deviceIndex  (0xFF for the Lift over BLE — confirmed by btsnoop)
 *  out19[1] = featureIndex
 *  out19[2] = (functionIndex << 4) | softwareId
 *  out19[3..18] = parameters, zero-padded
 */
static void build_long(uint8_t *out19,
                       uint8_t device_idx,
                       uint8_t feature_idx,
                       uint8_t func_idx,
                       const uint8_t *params, uint8_t params_len)
{
    memset(out19, 0, 19);
    out19[0] = device_idx;
    out19[1] = feature_idx;
    out19[2] = (uint8_t)((func_idx << 4) | HIDPP_SW_ID);
    if (params && params_len > 0) {
        uint8_t copy_len = (params_len > 16) ? 16 : params_len;
        memcpy(out19 + 3, params, copy_len);
    }
}

/* -------------------------------------------------------------------------
 * Helpers — GATT write
 * ---------------------------------------------------------------------- */

/*
 * Write callback for bt_gatt_write().  We don't need to act on the Write
 * Response itself — the HID++ reply arrives separately as a NOTIFY on
 * char002b.  We just log errors.
 */
static void hidpp_write_cb(struct bt_conn *conn, uint8_t err,
                            struct bt_gatt_write_params *params)
{
    if (err) {
        LOG_ERR("hidpp GATT write err=%u handle=0x%04x", err, params->handle);
    }
}

/*
 * Write a HID++ long message (19 payload bytes, no report-ID prefix).
 *
 * Uses bt_gatt_write() (ATT Write Request, opcode 0x12) rather than
 * bt_gatt_write_without_response() (opcode 0x52).  Confirmed by btsnoop
 * capture of logid: the Lift responds to Write Request (opcode 0x13 ack)
 * and only then sends the HID++ notification.  Write Command (0x52) may
 * be silently ignored.
 *
 * bt_gatt_write() requires a persistent bt_gatt_write_params struct for the
 * duration of the async call.  We allocate one per connection slot.
 */
static struct bt_gatt_write_params s_write_params[HIDPP_MAX_CONN];
static uint8_t s_write_buf[HIDPP_MAX_CONN][19];

static int hidpp_write_long(uint8_t conn_idx,
                             struct bt_conn *conn,
                             uint16_t handle,
                             const uint8_t *payload19)
{
    if (!conn || handle == 0) {
        LOG_ERR("hidpp_write_long: bad conn or handle=0x%04x", handle);
        return -EINVAL;
    }
    if (conn_idx >= HIDPP_MAX_CONN) {
        return -EINVAL;
    }

    /* Copy payload into persistent buffer (required by bt_gatt_write) */
    memcpy(s_write_buf[conn_idx], payload19, 19);

    s_write_params[conn_idx].func   = hidpp_write_cb;
    s_write_params[conn_idx].handle = handle;
    s_write_params[conn_idx].offset = 0;
    s_write_params[conn_idx].data   = s_write_buf[conn_idx];
    s_write_params[conn_idx].length = 19;

    int err = bt_gatt_write(conn, &s_write_params[conn_idx]);
    if (err) {
        LOG_ERR("bt_gatt_write err=%d handle=0x%04x", err, handle);
    }
    return err;
}

/* -------------------------------------------------------------------------
 * Step 1 — GetFeature(0x1b04)
 *
 * Over USB, HID++ GetFeature is a short (0x10) report.  Over BLE the Lift
 * exposes only a single Output Report characteristic with reportId=0x11
 * (confirmed by reading Report Reference descriptor 0x0031: value 0x11 0x02).
 * There is no 0x10 Output characteristic at all.  We therefore send GetFeature
 * as a long (0x11) report; the payload layout is identical, with the extra
 * bytes zero-padded.
 *
 * Device index: 0xFF, confirmed by btsnoop capture of logid communicating
 * with the Lift over BLE.  Despite being a direct BT connection (not a
 * Unifying receiver), the Lift uses 0xFF as the device index in both
 * directions — logid's writes use 0xFF and the Lift's notifications echo 0xFF.
 * ---------------------------------------------------------------------- */

static void send_get_feature(uint8_t conn_idx, struct bt_conn *conn)
{
    HidppConn &c = s_conns[conn_idx];

    /*
     * IRoot.GetFeature(featureCode=0x1b04)
     * Long report payload (19 bytes written to out_handle):
     *   [0] deviceIndex = 0xFF  (confirmed by btsnoop: Lift uses 0xFF over BLE)
     *   [1] featureIndex = 0x00 (IRoot)
     *   [2] funcByte = (funcIdx<<4) | swId = 0x01
     *   [3] 0x1b  feature code high
     *   [4] 0x04  feature code low
     *   [5..18] 0x00
     */
    uint8_t params[2] = {
        (uint8_t)(HIDPP_FEAT_REPROG_CTLS >> 8),   /* 0x1b */
        (uint8_t)(HIDPP_FEAT_REPROG_CTLS & 0xFF),  /* 0x04 */
    };
    uint8_t payload[19];
    build_long(payload,
               /*device_idx=*/0xFF,
               /*feature_idx=*/HIDPP_FEAT_IROOT_IDX,
               /*func_idx=*/HIDPP_FUNC_GET_FEATURE,
               params, sizeof(params));

    LOG_INF("conn[%u] hidpp: -> GetFeature(0x1b04) [long, device_idx=0xFF]",
            conn_idx);

    if (hidpp_write_long(conn_idx, conn, c.out_handle, payload) == 0) {
        c.state = HidppState::PROBING;
    } else {
        c.state = HidppState::IDLE;
    }
}

/* -------------------------------------------------------------------------
 * Step 2 — SetControlReporting(CID=0xfd, flags=TemporaryDiverted)
 * ---------------------------------------------------------------------- */

static void send_set_control_reporting(uint8_t conn_idx, struct bt_conn *conn,
                                       uint8_t feature_idx,
                                       uint16_t cid, uint8_t flags)
{
    HidppConn &c = s_conns[conn_idx];

    /*
     * SetControlReporting is function 3 of 0x1b04.
     * Long report payload layout (bytes after the 3-byte header):
     *   [0] CID high byte
     *   [1] CID low byte
     *   [2] flags
     *   [3..15] reserved / zero
     *
     * Both short (0x10) and long (0x11) HID++ writes go to the same
     * output characteristic handle (confirmed by GATT dump: the Lift
     * exposes only one writable Report char, at handle 0x0030).
     */
    uint8_t params[3] = {
        (uint8_t)(cid >> 8),
        (uint8_t)(cid & 0xFF),
        flags,
    };
    uint8_t payload[19];
    build_long(payload,
               /*device_idx=*/0xFF,
               feature_idx,
               HIDPP_FUNC_SET_CTRL_REPORTING,
               params, sizeof(params));

    LOG_INF("conn[%u] hidpp: -> SetControlReporting(CID=0x%04x, flags=0x%02x)",
            conn_idx, cid, flags);

    if (hidpp_write_long(conn_idx, conn, c.out_handle, payload) == 0) {
        c.state = HidppState::DIVERTING;
    } else {
        LOG_ERR("conn[%u] hidpp: SetControlReporting write failed", conn_idx);
        c.state = HidppState::IDLE;
    }
}

/* -------------------------------------------------------------------------
 * Public: hidpp_init
 * ---------------------------------------------------------------------- */

void hidpp_init(void)
{
    memset(s_conns, 0, sizeof(s_conns));
}

/* -------------------------------------------------------------------------
 * Public: hidpp_discovery
 * Walk the GATT DM attribute list to find the single OUTPUT Report
 * characteristic — the one Report (0x2a4d) char that has a Report Reference
 * descriptor (0x2908) but NO Client Characteristic Configuration descriptor
 * (0x2902).  Confirmed by bluetoothctl list-attributes on the Logitech Lift:
 *
 *   char0027 / value=0x0028:  CCCD(0x0029) + ReportRef(0x002a)  → INPUT
 *   char002b / value=0x002c:  CCCD(0x002d) + ReportRef(0x002e)  → INPUT
 *   char002f / value=0x0030:  ReportRef(0x0031),  NO CCCD       → OUTPUT ✓
 *
 * Both short (0x10) and long (0x11) HID++ writes go to handle 0x0030.
 * The Lift does not expose separate OUTPUT characteristics per report ID.
 * ---------------------------------------------------------------------- */

void hidpp_discovery(struct bt_gatt_dm *dm, uint8_t conn_idx)
{
    if (conn_idx >= HIDPP_MAX_CONN) {
        return;
    }

    HidppConn &c = s_conns[conn_idx];
    c.out_handle = 0;

    /* UUID for characteristic declaration (0x2803) */
    static const struct bt_uuid_16 uuid_char_decl =
        BT_UUID_INIT_16(0x2803);

    /*
     * Per-group tracking state.
     * A "group" is one characteristic declaration + its value + its descriptors.
     * We reset these whenever we see a new characteristic declaration (0x2803).
     */
    uint16_t grp_value_handle = 0;  /* value handle of current Report char    */
    bool     grp_is_report    = false; /* current char UUID is 0x2a4d?         */
    bool     grp_has_cccd     = false; /* seen 0x2902 in this group?           */
    bool     grp_has_rref     = false; /* seen 0x2908 in this group?           */

    /*
     * Inline lambda to evaluate a completed group and record the output handle
     * if it qualifies.  Called at the start of each new char declaration and
     * once more after the loop ends to handle the last group.
     */
    auto finalise_group = [&]() {
        if (grp_is_report && grp_has_rref && !grp_has_cccd &&
            grp_value_handle != 0 && c.out_handle == 0) {
            c.out_handle = grp_value_handle;
            LOG_INF("conn[%u] hidpp: output Report char value handle=0x%04x",
                    conn_idx, c.out_handle);
        }
    };

    const struct bt_gatt_dm_attr *attr = NULL;

    while (NULL != (attr = bt_gatt_dm_attr_next(dm, attr))) {

        if (attr->uuid->type != BT_UUID_TYPE_16) {
            continue;  /* vendor 128-bit UUIDs are not relevant here */
        }

        uint16_t uuid16 = BT_UUID_16(attr->uuid)->val;

        if (uuid16 == BT_UUID_16(&uuid_char_decl.uuid)->val) {
            /* New characteristic declaration — evaluate the group we just finished */
            finalise_group();
            /* Reset for the new group */
            grp_value_handle = attr->handle + 1; /* value is always decl+1 */
            grp_is_report    = false;
            grp_has_cccd     = false;
            grp_has_rref     = false;
            continue;
        }

        if (uuid16 == BT_UUID_16(&UUID_REPORT.uuid)->val) {
            /* Characteristic value attr for a Report (0x2a4d) char */
            grp_is_report    = true;
            grp_value_handle = attr->handle;  /* confirm (should equal decl+1) */
            continue;
        }

        if (uuid16 == BT_UUID_16(&UUID_CCCD.uuid)->val) {
            grp_has_cccd = true;
            continue;
        }

        if (uuid16 == BT_UUID_16(&UUID_REPORT_REF.uuid)->val) {
            grp_has_rref = true;
            continue;
        }
    }

    /* Finalise the last group in the attr list */
    finalise_group();

    if (c.out_handle != 0) {
        LOG_INF("conn[%u] hidpp: HID++ output handle=0x%04x "
                "(no-CCCD Report char)", conn_idx, c.out_handle);
    } else {
        LOG_DBG("conn[%u] hidpp: no HID++ output characteristic found "
                "(device may not support HID++)", conn_idx);
    }
}

/* -------------------------------------------------------------------------
 * Public: hidpp_on_ready
 * ---------------------------------------------------------------------- */

void hidpp_on_ready(uint8_t conn_idx, struct bt_conn *conn)
{
    if (conn_idx >= HIDPP_MAX_CONN) {
        return;
    }

    HidppConn &c = s_conns[conn_idx];

    if (c.out_handle == 0) {
        /* No OUTPUT Report characteristic found during discovery — not HID++ */
        c.state = HidppState::IDLE;
        return;
    }

    c.dpi_pressed        = false;
    c.descriptor_patched = false;
    c.feature_idx        = 0;

    send_get_feature(conn_idx, conn);
}

/* -------------------------------------------------------------------------
 * Public: hidpp_on_disconnect
 * ---------------------------------------------------------------------- */

void hidpp_on_disconnect(uint8_t conn_idx)
{
    if (conn_idx >= HIDPP_MAX_CONN) {
        return;
    }
    LOG_INF("conn[%u] hidpp: reset", conn_idx);
    memset(&s_conns[conn_idx], 0, sizeof(HidppConn));
}

/* -------------------------------------------------------------------------
 * Internal: handle GetFeature response (PROBING → DIVERTING)
 *
 * Short report (report_id=0x10) format (payload without report-ID byte):
 *   [0] deviceIndex
 *   [1] featureIndex (IRoot = 0x00)
 *   [2] funcByte = (funcIdx<<4 | swId)  — 0x01 for GetFeature response
 *   [3] featureIndex of 0x1b04 (0x00 = not found)
 *   [4] obsolete
 *   [5] hidden
 * ---------------------------------------------------------------------- */

static void handle_get_feature_response(uint8_t conn_idx,
                                        struct bt_conn *conn,
                                        const uint8_t *payload, uint8_t len)
{
    HidppConn &c = s_conns[conn_idx];

    if (len < 6) {
        LOG_WRN("conn[%u] hidpp: GetFeature response too short (%u)", conn_idx, len);
        c.state = HidppState::IDLE;
        return;
    }

    uint8_t feat_idx = payload[3];

    if (feat_idx == 0x00) {
        LOG_INF("conn[%u] hidpp: device does not support 0x1b04 (ReprogControls)",
                conn_idx);
        c.state = HidppState::IDLE;
        return;
    }

    c.feature_idx = feat_idx;
    LOG_INF("conn[%u] hidpp: 0x1b04 feature index = 0x%02x", conn_idx, feat_idx);

    /* Immediately divert CID 0xfd (DPI button) */
    send_set_control_reporting(conn_idx, conn, feat_idx,
                               HIDPP_CID_DPI,
                               HIDPP_FLAG_TEMPORARY_DIVERT);
}

/* -------------------------------------------------------------------------
 * Internal: handle SetControlReporting ack (DIVERTING → ACTIVE)
 *
 * Long report (report_id=0x11) format:
 *   [0] deviceIndex
 *   [1] featureIndex
 *   [2] funcByte = (3<<4 | swId) = 0x31
 *   [3] CID high
 *   [4] CID low
 *   [5] flags echoed
 * ---------------------------------------------------------------------- */

static void handle_set_ctrl_reporting_ack(uint8_t conn_idx,
                                          const uint8_t *payload, uint8_t len)
{
    HidppConn &c = s_conns[conn_idx];

    if (len < 6) {
        LOG_WRN("conn[%u] hidpp: SetControlReporting ack too short (%u)",
                conn_idx, len);
        c.state = HidppState::IDLE;
        return;
    }

    uint16_t cid   = ((uint16_t)payload[3] << 8) | payload[4];
    uint8_t  flags = payload[5];

    LOG_INF("conn[%u] hidpp: SetControlReporting ack CID=0x%04x flags=0x%02x",
            conn_idx, cid, flags);

    if (cid == HIDPP_CID_DPI) {
        c.state = HidppState::ACTIVE;
        LOG_INF("conn[%u] hidpp: CID 0x%04x diverted — DPI button now mappable",
                conn_idx, cid);
    } else {
        LOG_WRN("conn[%u] hidpp: unexpected CID 0x%04x in ack", conn_idx, cid);
        c.state = HidppState::IDLE;
    }
}

/* -------------------------------------------------------------------------
 * Internal: handle DivertedButtonsEvent (ACTIVE state)
 *
 * Long report (report_id=0x11) format:
 *   [0] deviceIndex
 *   [1] featureIndex
 *   [2] 0x00  (event, not a function response)
 *   [3] CID1 high
 *   [4] CID1 low
 *   [5] CID2 high
 *   [6] CID2 low
 *   [7] CID3 high
 *   [8] CID3 low
 *   [9] CID4 high
 *   [10] CID4 low
 *   (remaining bytes zero)
 *
 * A CID is "pressed" if it appears in slots 1..4; it is released when it
 * disappears from all slots in a subsequent event.
 * ---------------------------------------------------------------------- */

static void handle_diverted_buttons_event(uint8_t conn_idx,
                                          const uint8_t *payload, uint8_t len,
                                          hidpp_report_cb_t cb)
{
    HidppConn &c = s_conns[conn_idx];

    if (len < 11) {
        LOG_WRN("conn[%u] hidpp: DivertedButtonsEvent too short (%u)",
                conn_idx, len);
        return;
    }

    /* Scan the four CID slots */
    bool dpi_now_pressed = false;
    for (int slot = 0; slot < 4; slot++) {
        uint16_t cid = ((uint16_t)payload[3 + slot * 2] << 8) |
                        payload[4 + slot * 2];
        if (cid == HIDPP_CID_DPI) {
            dpi_now_pressed = true;
        }
    }

    if (dpi_now_pressed != c.dpi_pressed) {
        c.dpi_pressed = dpi_now_pressed;

        uint8_t button_byte = dpi_now_pressed ? 0x01u : 0x00u;
        LOG_INF("conn[%u] hidpp: DPI button %s",
                conn_idx, dpi_now_pressed ? "PRESSED" : "RELEASED");

        if (cb) {
            cb(conn_idx, HIDPP_SYNTH_REPORT_ID, &button_byte, 1);
        }
    }
}

/* -------------------------------------------------------------------------
 * Public: hidpp_handle_input_report
 * ---------------------------------------------------------------------- */

bool hidpp_handle_input_report(uint8_t conn_idx,
                               struct bt_conn *conn,
                               uint8_t report_id,
                               const uint8_t *payload,
                               uint8_t len,
                               hidpp_report_cb_t cb)
{
    if (conn_idx >= HIDPP_MAX_CONN) {
        return false;
    }

    /*
     * The Lift over BLE only exposes a single HID++ report channel: long
     * (0x11).  There is no 0x10 Input characteristic (confirmed by GATT dump:
     * char002b ReportRef = [0x11, Input]; no short-report Input char exists).
     * All HID++ traffic — commands, responses, and events — uses report 0x11.
     * We still guard against 0x10 here in case a different device is connected.
     */
    if (report_id != HIDPP_REPORT_SHORT && report_id != HIDPP_REPORT_LONG) {
        return false;
    }

    HidppConn &c = s_conns[conn_idx];

    if (c.state == HidppState::IDLE) {
        /*
         * We didn't find an output characteristic during discovery.
         * Pass through so handle_received_report sees it.
         */
        return false;
    }

    if (len < 3) {
        /* Too short to be a valid HID++ 2.0 message */
        return false;
    }

    /*
     * payload layout (after stripping report-ID):
     *   [0] deviceIndex  — not validated; BT is point-to-point
     *   [1] featureIndex (or 0xFF for error packets)
     *   [2] funcByte = (functionIndex<<4 | softwareId)
     *   [3..] params / event data
     */

    uint8_t feat_in_msg = payload[1];
    uint8_t func_byte   = payload[2];
    uint8_t func_idx    = func_byte >> 4;

    /* HID++ 2.0 error packets: featureIndex=0xFF */
    if (feat_in_msg == 0xFF) {
        LOG_WRN("conn[%u] hidpp: error packet func=0x%02x payload[3]=0x%02x",
                conn_idx, func_byte, len > 3 ? payload[3] : 0);
        return true;  /* consumed — don't pass to handle_received_report */
    }

    switch (c.state) {
        case HidppState::PROBING:
            /*
             * Waiting for GetFeature(0x1b04) response.
             *
             * Over BLE the Lift responds with a long (0x11) report even though
             * the USB implementation uses short (0x10) for GetFeature.
             * Match: report_id=0x11, featureIndex=0x00 (IRoot), funcIdx=0.
             */
            if (report_id == HIDPP_REPORT_LONG &&
                feat_in_msg == HIDPP_FEAT_IROOT_IDX &&
                func_idx == HIDPP_FUNC_GET_FEATURE) {
                handle_get_feature_response(conn_idx, conn, payload, len);
                return true;
            }
            /*
             * Any other 0x11 report while probing (e.g. a mouse movement
             * notification that somehow arrived first) — pass through.
             */
            return false;

        case HidppState::DIVERTING:
            /*
             * Waiting for SetControlReporting ack.
             * Expect: report 0x11, featureIndex=c.feature_idx, func=0x31.
             */
            if (report_id == HIDPP_REPORT_LONG &&
                feat_in_msg == c.feature_idx &&
                func_idx == HIDPP_FUNC_SET_CTRL_REPORTING) {
                handle_set_ctrl_reporting_ack(conn_idx, payload, len);
                return true;
            }
            return false;

        case HidppState::ACTIVE:
            /*
             * Diversion is live.  Intercept DivertedButtonsEvent:
             * report 0x11, featureIndex=c.feature_idx, funcByte=0x00 (event).
             */
            if (report_id == HIDPP_REPORT_LONG &&
                feat_in_msg == c.feature_idx &&
                func_byte == 0x00) {
                handle_diverted_buttons_event(conn_idx, payload, len, cb);
                return true;
            }
            /*
             * Other HID++ reports while active (e.g. other feature responses)
             * fall through to handle_received_report unchanged.
             */
            return false;

        default:
            return false;
    }
}

/* -------------------------------------------------------------------------
 * Public: hidpp_amend_descriptor
 *
 * Appends a synthetic HID report descriptor fragment declaring the DPI
 * button as Button 16, Report ID 0xFD.
 *
 * Fragment bytes (24 bytes):
 *
 *   05 09        Usage Page (Button)
 *   09 10        Usage (Button 16)        <- HIDPP_DPI_BUTTON_USAGE
 *   15 00        Logical Minimum (0)
 *   25 01        Logical Maximum (1)
 *   75 01        Report Size (1)
 *   95 01        Report Count (1)
 *   85 FD        Report ID (0xFD)         <- HIDPP_SYNTH_REPORT_ID
 *   81 02        Input (Data, Variable, Absolute)
 *   75 07        Report Size (7)           [padding to full byte]
 *   95 01        Report Count (1)
 *   81 03        Input (Constant, Variable, Absolute)
 *
 * Total: 22 bytes  (<= HIDPP_DESCRIPTOR_PATCH_SIZE=24, leaving 2 bytes slack)
 *
 * We only append if the connection has progressed past PROBING (i.e. we
 * confirmed the device supports 0x1b04) so we don't pollute non-HID++
 * device descriptors.
 * ---------------------------------------------------------------------- */

/* clang-format off */
static const uint8_t k_dpi_descriptor_patch[] = {
    0x05, 0x09,                        /* Usage Page (Button)             */
    0x09, HIDPP_DPI_BUTTON_USAGE,      /* Usage (Button 16)               */
    0x15, 0x00,                        /* Logical Minimum (0)             */
    0x25, 0x01,                        /* Logical Maximum (1)             */
    0x75, 0x01,                        /* Report Size (1)                 */
    0x95, 0x01,                        /* Report Count (1)                */
    0x85, HIDPP_SYNTH_REPORT_ID,       /* Report ID (0xFD)                */
    0x81, 0x02,                        /* Input (Data, Variable, Absolute)*/
    0x75, 0x07,                        /* Report Size (7)  -- padding     */
    0x95, 0x01,                        /* Report Count (1)                */
    0x81, 0x03,                        /* Input (Constant)                */
};
/* clang-format on */

static_assert(sizeof(k_dpi_descriptor_patch) <= HIDPP_DESCRIPTOR_PATCH_SIZE,
              "patch exceeds HIDPP_DESCRIPTOR_PATCH_SIZE");

uint16_t hidpp_amend_descriptor(uint8_t conn_idx,
                                uint8_t *descriptor,
                                uint16_t orig_size,
                                uint16_t buf_size)
{
    if (conn_idx >= HIDPP_MAX_CONN) {
        return orig_size;
    }

    HidppConn &c = s_conns[conn_idx];

    /*
     * Patch whenever we found a HID++ output characteristic during GATT
     * discovery (out_handle != 0).  We do NOT gate on having confirmed
     * 0x1b04 support yet, because the descriptor read races the GetFeature
     * round-trip: both are initiated from hogp_ready_work_fn and can complete
     * in either order.  If 0x1b04 turns out to be absent the probe sequence
     * will leave state at IDLE, no events will arrive for report 0xFD, and
     * the extra descriptor entry is harmless dead weight.
     */
    if (c.out_handle == 0) {
        return orig_size;
    }

    if (c.descriptor_patched) {
        /* Already patched (shouldn't happen, but be safe) */
        return orig_size;
    }

    uint16_t patch_len = (uint16_t)sizeof(k_dpi_descriptor_patch);

    if ((uint32_t)orig_size + patch_len > buf_size) {
        LOG_ERR("conn[%u] hidpp: descriptor buffer too small to patch "
                "(have %u, need %u)", conn_idx, buf_size, orig_size + patch_len);
        return orig_size;
    }

    memcpy(descriptor + orig_size, k_dpi_descriptor_patch, patch_len);
    c.descriptor_patched = true;

    LOG_INF("conn[%u] hidpp: appended %u-byte DPI button descriptor patch "
            "(new total=%u)", conn_idx, patch_len, orig_size + patch_len);

    return orig_size + patch_len;
}
