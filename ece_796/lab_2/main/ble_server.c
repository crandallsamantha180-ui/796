#include "ble_server.h"

#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "os/os_mbuf.h"
#include "store/config/ble_store_config.h"

static const char *TAG = "BLE_SERVER";

static uint16_t g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t g_sensor_char_handle;
static bool g_ble_ready = false;
static sensor_packet_t g_last_packet;
static uint8_t g_own_addr_type;

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg);

static int gatt_access_cb(uint16_t conn_handle, uint16_t attr_handle,
						  struct ble_gatt_access_ctxt *ctxt, void *arg)
{
	(void)conn_handle;
	(void)attr_handle;
	(void)arg;

	if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
		int rc = os_mbuf_append(ctxt->om, &g_last_packet, sizeof(g_last_packet));
		return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
	}

	return BLE_ATT_ERR_UNLIKELY;
}

static const ble_uuid128_t g_service_uuid = BLE_UUID128_INIT(
	0x2d, 0x1f, 0x28, 0x59, 0x62, 0x40, 0x4b, 0x2d,
	0x9a, 0x9c, 0xf3, 0x1a, 0x9c, 0x41, 0x0f, 0xa1
);

static const ble_uuid128_t g_char_uuid = BLE_UUID128_INIT(
	0x6a, 0x78, 0x11, 0x1b, 0x6d, 0x7a, 0x45, 0x5f,
	0xb3, 0x8e, 0x5c, 0x7a, 0x03, 0xb6, 0x66, 0x5a
);

static const struct ble_gatt_svc_def gatt_svcs[] = {
	{
		.type = BLE_GATT_SVC_TYPE_PRIMARY,
		.uuid = &g_service_uuid.u,
		.characteristics = (struct ble_gatt_chr_def[]){
			{
				.uuid = &g_char_uuid.u,
				.access_cb = gatt_access_cb,
				.flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
				.val_handle = &g_sensor_char_handle,
			},
			{
				0,
			},
		},
	},
	{
		0,
	},
};

static void ble_advertise(void)
{
	struct ble_hs_adv_fields fields;
	memset(&fields, 0, sizeof(fields));

	fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
	fields.name = (uint8_t *)ble_svc_gap_device_name();
	fields.name_len = strlen(ble_svc_gap_device_name());
	fields.name_is_complete = 1;

	int rc = ble_gap_adv_set_fields(&fields);
	if (rc != 0) {
		ESP_LOGE(TAG, "adv_set_fields failed: %d", rc);
		return;
	}

	struct ble_gap_adv_params adv_params;
	memset(&adv_params, 0, sizeof(adv_params));
	adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
	adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

	rc = ble_gap_adv_start(g_own_addr_type, NULL, BLE_HS_FOREVER,
						   &adv_params, ble_gap_event_cb, NULL);
	if (rc != 0) {
		ESP_LOGE(TAG, "adv_start failed: %d", rc);
	}
}

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg)
{
	(void)arg;

	switch (event->type) {
		case BLE_GAP_EVENT_CONNECT:
			if (event->connect.status == 0) {
				g_conn_handle = event->connect.conn_handle;
				ESP_LOGI(TAG, "Connected, handle=%u", g_conn_handle);
			} else {
				ESP_LOGW(TAG, "Connect failed; status=%d", event->connect.status);
				ble_advertise();
			}
			return 0;

		case BLE_GAP_EVENT_DISCONNECT:
			ESP_LOGI(TAG, "Disconnected; reason=%d", event->disconnect.reason);
			g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
			ble_advertise();
			return 0;

		case BLE_GAP_EVENT_ADV_COMPLETE:
			ESP_LOGI(TAG, "Advertising complete; restarting");
			ble_advertise();
			return 0;

		default:
			return 0;
	}
}

static void ble_on_reset(int reason)
{
	ESP_LOGE(TAG, "Resetting; reason=%d", reason);
}

static void ble_on_sync(void)
{
	uint8_t addr_val[6] = {0};
	int rc = ble_hs_id_infer_auto(0, &g_own_addr_type);
	if (rc != 0) {
		ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: %d", rc);
		return;
	}
	ble_hs_id_copy_addr(g_own_addr_type, addr_val, NULL);
	ESP_LOGI(TAG, "BLE address: %02X:%02X:%02X:%02X:%02X:%02X",
			 addr_val[5], addr_val[4], addr_val[3],
			 addr_val[2], addr_val[1], addr_val[0]);

	g_ble_ready = true;
	ble_advertise();
}

static void ble_host_task(void *param)
{
	(void)param;
	nimble_port_run();
	nimble_port_freertos_deinit();
}

void init_ble_stack(void)
{
	memset(&g_last_packet, 0, sizeof(g_last_packet));
	g_last_packet.header[0] = 0xAA;
	g_last_packet.header[1] = 0xAA;
	g_last_packet.header[2] = 0xAA;

	nimble_port_init();

	ble_hs_cfg.reset_cb = ble_on_reset;
	ble_hs_cfg.sync_cb = ble_on_sync;

	ble_svc_gap_init();
	ble_svc_gatt_init();

	ble_svc_gap_device_name_set("ESP32-IMU");

	int rc = ble_gatts_count_cfg(gatt_svcs);
	if (rc != 0) {
		ESP_LOGE(TAG, "gatt count cfg failed: %d", rc);
		return;
	}

	rc = ble_gatts_add_svcs(gatt_svcs);
	if (rc != 0) {
		ESP_LOGE(TAG, "gatt add svcs failed: %d", rc);
		return;
	}

	ble_hs_cfg.gatts_register_cb = NULL;

	nimble_port_freertos_init(ble_host_task);
}

bool ble_is_connected(void)
{
	return g_conn_handle != BLE_HS_CONN_HANDLE_NONE;
}

int ble_send_sensor_packet(const sensor_packet_t *packet)
{
	if (!g_ble_ready || g_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
		return BLE_HS_EINVAL;
	}

	memcpy(&g_last_packet, packet, sizeof(g_last_packet));

	struct os_mbuf *om = ble_hs_mbuf_from_flat(packet, sizeof(*packet));
	if (om == NULL) {
		return BLE_HS_ENOMEM;
	}

	return ble_gatts_notify_custom(g_conn_handle, g_sensor_char_handle, om);
}
