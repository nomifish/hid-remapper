/*
 * hidpp.h — HID++ 2.0 ReprogControls diversion for hid-remapper BT firmware
 *
 * On connection to a device that exposes HID report IDs 0x10/0x11 as output
 * characteristics, we probe for feature 0x1b04 (ReprogControls) and divert
 * the target CIDs listed in HIDPP_DIVERT_CIDS so that the device sends events
 * to us instead of acting on them internally.
 *
 * Incoming diverted-button events are translated into synthetic HID button
 * reports (report ID 0xFD) injected into the normal report queue so that
 * hid-remapper sees them as mappable source usages.
 *
 * To make hid-remapper aware of that synthetic report, call
 * hidpp_amend_descriptor() on the raw descriptor bytes before handing them
 * to parse_descriptor().
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/services/hogp.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * Tunables
 * ---------------------------------------------------------------------- */

/*
 * Maximum number of BT connections — must match CONFIG_BT_MAX_CONN in
 * prj.conf (currently 8).
 */
#define HIDPP_MAX_CONN  8

/*
 * Software ID embedded in HID++ function/software-ID byte.
 * 4-bit value 1-15; 0 is reserved for the device.  We use 1.
 */
#define HIDPP_SW_ID     0x01

/*
 * HID++ report IDs.
 *
 * Over USB, HID++ uses both 0x10 (short, 7 bytes) and 0x11 (long, 20 bytes).
 * Over BLE, the Logitech Lift exposes only a single Output Report characteristic
 * with reportId=0x11 (confirmed by GATT dump: desc0031 = [0x11, 0x02]).
 * There is no 0x10 Output characteristic.  All HID++ commands are sent as long
 * reports; all responses and events arrive as long Input notifications on the
 * 0x11 Input characteristic (char002b).
 *
 * HIDPP_REPORT_SHORT is kept as a filter constant in hidpp_handle_input_report
 * to pass through any stray 0x10 notifications from non-Lift devices.
 */
#define HIDPP_REPORT_SHORT  0x10   /* HID++ short — no Output char on Lift BLE */
#define HIDPP_REPORT_LONG   0x11   /* HID++ long  — sole Output/Input channel   */

/*
 * HID++ 2.0 feature 0x0000 — IRoot, function 0 — GetFeature.
 * funcByte = (functionIndex << 4) | swId
 */
#define HIDPP_FEAT_IROOT_IDX   0x00
#define HIDPP_FUNC_GET_FEATURE 0x00   /* function 0 of IRoot */
#define HIDPP_FEAT_REPROG_CTLS 0x1b04 /* ReprogControls V4 */

/*
 * ReprogControls functions.
 */
#define HIDPP_FUNC_SET_CTRL_REPORTING 0x03

/*
 * SetControlReporting flags byte.
 * bit 0 = Diverted (persistent across power cycles)
 * bit 1 = TemporaryDiverted (session only — resets on reconnect)
 * bit 5 = AnalyticsKeyEvt (optional, not needed here)
 */
#define HIDPP_FLAG_TEMPORARY_DIVERT 0x02

/*
 * CIDs we want to divert.  Only CIDs that do NOT already appear in the
 * standard HID descriptor need diverting; the others (middle/back/forward)
 * already reach hid-remapper through normal reports.
 */
#define HIDPP_CID_DPI 0x00fd

/*
 * The synthetic HID report ID we inject into the report queue when a
 * diverted button event arrives.  Must not collide with any report ID in
 * the device's own HID descriptor (the Lift uses 0x01–0x03; 0xFD is safe).
 */
#define HIDPP_SYNTH_REPORT_ID  0xFD

/*
 * The HID button usage number we assign to the DPI button in the synthetic
 * descriptor fragment.  Button 16 (0x10) is well above the 3–5 buttons a
 * standard mouse exposes.
 */
#define HIDPP_DPI_BUTTON_USAGE 0x10

/* -------------------------------------------------------------------------
 * Callback type
 * ---------------------------------------------------------------------- */

/*
 * Called by hidpp_handle_input_report() when a diverted button changes
 * state and a synthetic report should be injected.
 *
 * report_id   – HIDPP_SYNTH_REPORT_ID (0xFD)
 * payload     – single byte: 0x01 pressed, 0x00 released
 * len         – always 1
 * conn_idx    – BT connection index (0 .. HIDPP_MAX_CONN-1)
 */
typedef void (*hidpp_report_cb_t)(uint8_t conn_idx,
                                  uint8_t report_id,
                                  const uint8_t *payload,
                                  uint8_t len);

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

/**
 * hidpp_init() — zero out all per-connection state.
 * Call once before the BT stack starts.
 */
void hidpp_init(void);

/**
 * hidpp_discovery() — walk GATT DM attrs to find the output/feature report
 * characteristic handles for report IDs 0x10 and 0x11.
 *
 * Call this inside discovery_completed_cb(), BEFORE bt_hogp_handles_assign().
 * We need the DM attrs while they are still valid.
 *
 * @param dm        Completed GATT discovery manager context.
 * @param conn_idx  BT connection index for this device.
 */
void hidpp_discovery(struct bt_gatt_dm *dm, uint8_t conn_idx);

/**
 * hidpp_on_ready() — begin the HID++ probe sequence.
 *
 * Call from hogp_ready_work_fn() after device_connected_callback(), once for
 * each newly-ready hogp.  If the device has no 0x10 output handle (found
 * during hidpp_discovery) this is a no-op.
 *
 * @param conn_idx  BT connection index.
 * @param conn      Active bt_conn for this device.
 */
void hidpp_on_ready(uint8_t conn_idx, struct bt_conn *conn);

/**
 * hidpp_on_disconnect() — reset per-connection HID++ state.
 *
 * Call from the disconnected handler.
 *
 * @param conn_idx  BT connection index.
 */
void hidpp_on_disconnect(uint8_t conn_idx);

/**
 * hidpp_handle_input_report() — intercept HID++ messages arriving via
 * hogp_notify_cb before they reach handle_received_report().
 *
 * @param conn_idx   BT connection index.
 * @param conn       Active bt_conn (needed for GATT writes during setup).
 * @param report_id  The HID report ID (first byte that main.cc prefixes).
 * @param payload    Report payload bytes (WITHOUT the report-ID byte).
 * @param len        Length of payload.
 * @param cb         Called when a synthetic report should be injected.
 *
 * @return true   if the report was a HID++ message and was fully consumed
 *                (caller must NOT pass it to handle_received_report()).
 * @return false  if this is an ordinary HID report (caller proceeds normally).
 */
bool hidpp_handle_input_report(uint8_t conn_idx,
                               struct bt_conn *conn,
                               uint8_t report_id,
                               const uint8_t *payload,
                               uint8_t len,
                               hidpp_report_cb_t cb);

/**
 * hidpp_amend_descriptor() — append a synthetic HID descriptor fragment that
 * declares the diverted DPI button as a mappable source.
 *
 * Call this on the raw descriptor bytes received from hogp_map_read_cb(),
 * BEFORE calling parse_descriptor().
 *
 * The function appends only when conn_idx has successfully reached at least
 * the WAIT_DIVERT_ACK state (i.e. we confirmed the device has 0x1b04).  For
 * non-HID++ devices it is a no-op and returns orig_size unchanged.
 *
 * @param conn_idx   BT connection index.
 * @param descriptor Buffer containing the raw descriptor; must have room for
 *                   at least HIDPP_DESCRIPTOR_PATCH_SIZE extra bytes.
 * @param orig_size  Current size of the descriptor.
 * @param buf_size   Total capacity of the descriptor buffer.
 *
 * @return New size of the descriptor (orig_size + patch bytes, or orig_size).
 */
uint16_t hidpp_amend_descriptor(uint8_t conn_idx,
                                uint8_t *descriptor,
                                uint16_t orig_size,
                                uint16_t buf_size);

/**
 * Worst-case number of bytes appended by hidpp_amend_descriptor().
 * The caller's descriptor buffer must have at least this much headroom.
 */
#define HIDPP_DESCRIPTOR_PATCH_SIZE 24

#ifdef __cplusplus
}
#endif
