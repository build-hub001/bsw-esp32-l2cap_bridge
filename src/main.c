#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/portmacro.h>
#include <freertos/event_groups.h>

#include <sys/stat.h>
#include <sys/param.h>

#include <soc/timer_group_struct.h>
#include <soc/timer_group_reg.h>
#include <soc/rtc.h>

#include <esp_task_wdt.h>
#include <esp_system.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>

#include <esp_system.h>
#include <nvs_flash.h>
#include <nvs.h>

#include "driver/uart.h"
#include "driver/gpio.h"

#include "hid_l2cap.h"

static const char *TAG = "app";

uint8_t PS5_DUALSHOCK_BT_ADDR[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

#define PS5_DUALSHOCCK_RECONNECT_MS 6*1000

#define UART_TXD           (32)
#define UART_RXD           (33)
#define UART_RTS           (UART_PIN_NO_CHANGE)
#define UART_CTS           (UART_PIN_NO_CHANGE)
#define UART_PORT_NUM      (1)
#define UART_BAUD_RATE     (115200)
#define UART_BUF_SIZE      (1024)

void l2cap_data_callback(uint8_t *p_msg, size_t len)
{
    uart_write_bytes( UART_PORT_NUM, (const char *) p_msg, len);
}

int64_t millis()
{
    return esp_timer_get_time() / 1000;
}

/*
-------------------------------------------------------------------------------

    @enter_point

-------------------------------------------------------------------------------
*/
extern void app_main(void)
{
/* 
* 
* Initialize NVS
*/
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

/* 
* Initialize Bluetooth + L2CAP
* 
*/
    esp_bt_gap_set_scan_mode( ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE );

    long ret = hid_l2cap_initialize( l2cap_data_callback );
    if (ret != 0){
        while (1){
            ESP_LOGE( TAG, "failed initializing l2cap");
            vTaskDelay(3 * 1000 / portTICK_PERIOD_MS);
        }
    }

    ret = hid_l2cap_connect(PS5_DUALSHOCK_BT_ADDR);
    if (ret != 0){
        ESP_LOGI("", "failed connection");
    }


/* 
* Initialize UART driver,
* Configure communication pins and install the driver 
*/
    uart_config_t uart_config = 
    {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK( uart_driver_install( UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK( uart_param_config( UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK( uart_set_pin( UART_PORT_NUM, UART_TXD, UART_RXD, UART_RTS, UART_CTS));

    uint32_t start_tim = 0;
    loop:while(true)
    {
        enBT_STATUS bt_status = hid_l2cap_get_status();

        if ( BT_CONNECTING == bt_status)
        {
            uint32_t tdelta = millis()-start_tim;
            if ( tdelta >= PS5_DUALSHOCCK_RECONNECT_MS)
            {
                start_tim = tdelta;
                hid_l2cap_reconnect();
            }
        }
        else if ( BT_DISCONNECTED == bt_status )
        {
            ESP_LOGI("", "disconnected");
            start_tim = millis();
            hid_l2cap_reconnect();
        }
        else
        {

        }

        vTaskDelay(25 / portTICK_PERIOD_MS);
    }
}