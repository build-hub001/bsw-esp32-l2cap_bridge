#include <esp_log.h>

#include "osi/allocator.h"
#include "stack/l2c_api.h"
#include "stack/btm_api.h"
#include "hid_l2cap.h"

#define HID_L2CAP_ID_HIDC 0x40
#define HID_L2CAP_ID_HIDI 0x41

#define TAG "hid_l2cap"

static long hid_l2cap_init_service(const char *name, uint16_t psm, uint8_t security_id);
static void hid_l2cap_deinit_service(const char *name, uint16_t psm);

static void hid_l2cap_connect_cfm_cback(uint16_t l2cap_cid, uint16_t result);
static void hid_l2cap_config_ind_cback(uint16_t l2cap_cid, tL2CAP_CFG_INFO *p_cfg);
static void hid_l2cap_config_cfm_cback(uint16_t l2cap_cid, tL2CAP_CFG_INFO *p_cfg);
static void hid_l2cap_disconnect_ind_cback(uint16_t l2cap_cid, bool ack_needed);
static void hid_l2cap_disconnect_cfm_cback(uint16_t l2cap_cid, uint16_t result);
static void hid_l2cap_data_ind_cback(uint16_t l2cap_cid, BT_HDR *p_msg);

static void dump_bin(const char *p_message, const uint8_t *p_bin, int len);

bool bt_Started();
bool bt_Start();

static BD_ADDR g_bd_addr;
static enBT_STATUS is_connected = BT_UNINITIALIZED;
static HID_L2CAP_CALLBACK g_callback;

static uint16_t l2cap_cid_hidc;
static uint16_t l2cap_cid_hidi;

static tL2CAP_ERTM_INFO hid_ertm_info;
static tL2CAP_CFG_INFO hid_cfg_info;

void hid_l2cap_connect_ind_cb(uint8_t *address, uint16_t localCid, uint16_t psm, uint8_t identifier)
{
    /* Send connection pending response to the L2CAP layer. */
    L2CA_CONNECT_RSP(address, identifier, localCid, L2CAP_CONN_PENDING, L2CAP_CONN_PENDING, NULL, NULL);

    /* Send response to the L2CAP layer. */
    L2CA_CONNECT_RSP(address, identifier, localCid, L2CAP_CONN_OK, L2CAP_CONN_OK, NULL, NULL);

    /* Send a Configuration Request. */
    L2CA_CONFIG_REQ(localCid, &hid_cfg_info);

    if (psm == BT_PSM_HIDC)
    {
        l2cap_cid_hidc = localCid;
    }
    else if (psm == BT_PSM_HIDI)
    {
        l2cap_cid_hidi = localCid;
    }

    ESP_LOGI(TAG, "hid_l2cap_connect_ind_cb");
}

void hid_l2cap_connect_pnd_cb(uint16_t cid)
{
    ESP_LOGI(TAG, "pending connection cid: %d", cid);
}

// tL2CA_QOS_VIOLATION_IND_CB;
void hid_l2cap_qos_violation_ind_cb(uint8_t *address)
{
    ESP_LOGI(TAG, "hid violation");
}

// tL2CA_CONGESTION_STATUS_CB;
void hid_l2cap_congestion_status_cb(uint16_t cid, bool congested)
{
    ESP_LOGI(TAG, "conguestion status cb cid: String(cid)  conguested: congested");
}

// tL2CA_TX_COMPLETE_CB;
void hid_l2cap_tx_complete_cb(uint16_t cid, uint16_t numberOfSdu)
{
    // ESP_LOGI(TAG,"tx complete cid:" + String(cid)  " number of sdu: " + String(numberOfSdu));
}

static const tL2CAP_APPL_INFO dyn_info = 
{
    hid_l2cap_connect_ind_cb, hid_l2cap_connect_cfm_cback,
    hid_l2cap_connect_pnd_cb, hid_l2cap_config_ind_cback,
    hid_l2cap_config_cfm_cback, hid_l2cap_disconnect_ind_cback,
    hid_l2cap_disconnect_cfm_cback, hid_l2cap_qos_violation_ind_cb,
    hid_l2cap_data_ind_cback, hid_l2cap_congestion_status_cb,
    hid_l2cap_tx_complete_cb
};

static long hid_l2cap_init_services(void)
{
    long ret;
    ret = hid_l2cap_init_service("HIDC", BT_PSM_HIDC, BTM_SEC_SERVICE_FIRST_EMPTY);
    if (ret != 0)
        return ret;
    ret = hid_l2cap_init_service("HIDI", BT_PSM_HIDI,
                                 BTM_SEC_SERVICE_FIRST_EMPTY + 1);
    if (ret != 0)
        return ret;

    return 0;
}

static void hid_l2cap_deinit_services(void)
{
    hid_l2cap_deinit_service("HIDC", BT_PSM_HIDC);
    hid_l2cap_deinit_service("HIDI", BT_PSM_HIDI);
}

static long hid_l2cap_init_service(const char *name, uint16_t psm,
                                   uint8_t security_id)
{
    /* Register the PSM for incoming connections */
    if (!L2CA_Register(psm, (tL2CAP_APPL_INFO *)&dyn_info))
    {
        ESP_LOGI(TAG, "%s Registering service %s failed\n", __func__, name);
        return -1;
    }

    /* Register with the Security Manager for our specific security level (none)
     */
    if (!BTM_SetSecurityLevel(false, name, security_id, 0, psm, 0, 0))
    {
        ESP_LOGI(TAG, "%s Registering security service %s failed\n", __func__, name);
        return -1;
    }

    ESP_LOGI(TAG, "[%s] Service %s Initialized\n", __func__, name);

    return 0;
}

static void hid_l2cap_deinit_service(const char *name, uint16_t psm)
{
    L2CA_Deregister(psm);
    ESP_LOGI(TAG, "[%s] Service %s Deinitialized\n", __func__, name);
}

enBT_STATUS hid_l2cap_get_status(void) { return is_connected; }

long hid_l2cap_reconnect(void)
{
    long ret;
    ret = L2CA_CONNECT_REQ(BT_PSM_HIDC, g_bd_addr, NULL, NULL);
    ESP_LOGI(TAG, "L2CA_CONNECT_REQ ret=%d\n", ret);
    if (ret == 0)
    {
        return -1;
    }
    l2cap_cid_hidc = ret;

    is_connected = BT_CONNECTING;

    return ret;
}

long hid_l2cap_connect(BD_ADDR addr)
{
    memmove(g_bd_addr, addr, sizeof(BD_ADDR));

    return hid_l2cap_reconnect();
}

long hid_l2cap_initialize(HID_L2CAP_CALLBACK callback)
{
    if (!bt_Started() && !bt_Start())
    {
        ESP_LOGI(TAG, "bt_Start failed");
        return -1;
    }

    esp_bluedroid_status_t bt_state = esp_bluedroid_get_status();
    if (bt_state == ESP_BLUEDROID_STATUS_UNINITIALIZED)
    {
        if (esp_bluedroid_init())
        {
            ESP_LOGI(TAG, "esp_bluedroid_init failed");
            return -1;
        }
    }

    if (bt_state != ESP_BLUEDROID_STATUS_ENABLED)
    {
        if (esp_bluedroid_enable())
        {
            ESP_LOGI(TAG, "esp_bluedroid_enable failed");
            return -1;
        }
    }

    if (hid_l2cap_init_services() != 0)
    {
        ESP_LOGI(TAG, "hid_l2cap_init_services failed");
        return -1;
    }

    g_callback = callback;

    is_connected = BT_DISCONNECTED;

    return 0;
}

static void hid_l2cap_connect_cfm_cback(uint16_t l2cap_cid, uint16_t result)
{
    ESP_LOGI(TAG, "[%s] l2cap_cid: 0x%02x\n  result: %d\n", __func__, l2cap_cid,
             result);
}

static void hid_l2cap_config_cfm_cback(uint16_t l2cap_cid,
                                       tL2CAP_CFG_INFO *p_cfg)
{
    ESP_LOGI(TAG, "[%s] l2cap_cid: 0x%02x\n  p_cfg->result: %d\n", __func__,
             l2cap_cid, p_cfg->result);

    if (l2cap_cid == l2cap_cid_hidc)
    {
    }
    else if (l2cap_cid == l2cap_cid_hidi)
    {
        is_connected = BT_CONNECTED;

        ESP_LOGI(TAG, "Hid Connected");
    }
}

static void hid_l2cap_config_ind_cback(uint16_t l2cap_cid,
                                       tL2CAP_CFG_INFO *p_cfg)
{
    ESP_LOGI(TAG,
             "[%s] l2cap_cid: 0x%02x\n  p_cfg->result: %d\n  p_cfg->mtu_present: %d\n "
             " p_cfg->mtu: %d\n",
             __func__, l2cap_cid, p_cfg->result, p_cfg->mtu_present, p_cfg->mtu);

    p_cfg->result = L2CAP_CFG_OK;

    L2CA_ConfigRsp(l2cap_cid, p_cfg);
}

static void hid_l2cap_disconnect_ind_cback(uint16_t l2cap_cid,
                                           bool ack_needed)
{
    ESP_LOGI(TAG, "[%s] l2cap_cid: 0x%02x\n  ack_needed: %d\n", __func__,
             l2cap_cid, ack_needed);
    is_connected = BT_DISCONNECTED;
    //todo: need fix - zero after reconnect
    // g_callback = NULL;
}

static void hid_l2cap_disconnect_cfm_cback(uint16_t l2cap_cid,
                                           uint16_t result)
{
    ESP_LOGI(TAG, "[%s] l2cap_cid: 0x%02x\n  result: %d\n", __func__, l2cap_cid,
             result);
}

unsigned long printedAt = 0;
static void hid_l2cap_data_ind_cback(uint16_t l2cap_cid, BT_HDR *p_buf)
{
    if (millis() - printedAt > 50UL)
    {
        printedAt = millis();
        // Serial.printf("[%s] l2cap_cid: 0x%02x\n", __func__, l2cap_cid);
        // Serial.printf("event=%d len=%d offset=%d layer_specific=%d\n", p_buf->event,
        //               p_buf->len, p_buf->offset, p_buf->layer_specific);
        dump_bin("\tdata = ", &p_buf->data[p_buf->offset], p_buf->len);

        if (g_callback != NULL)
            g_callback(&p_buf->data[p_buf->offset], p_buf->len);
    }

    osi_free(p_buf);
}

static void dump_bin(const char *p_message, const uint8_t *p_bin, int len)
{
    for (int i = 0; i < len; i++)
    {
        printf("%02x ", p_bin[i]);
    }
    printf( "\n");
}

bool bt_Started()
{
    return (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED);
}

bool bt_Start()
{
    esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED)
    {
        return true;
    }

    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE)
    {
        esp_bt_controller_init(&cfg);
        while (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE)
        {
        }
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED)
    {
        if (esp_bt_controller_enable(ESP_BT_MODE_BTDM))
        {
            ESP_LOGE(TAG, "BT Enable failed");
            return false;
        }
    }
    
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED)
    {
        return true;
    }

    ESP_LOGE(TAG, "BT Start failed");
    return false;
}