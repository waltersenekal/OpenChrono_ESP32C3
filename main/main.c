#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_mac.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"



#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define GATTS_TAG "GATTS"

///Declare the static function
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

#define GATTS_SERVICE_CHRONO_UUID   0x00FF
#define GATTS_CHAR_CHRONO_UUID      0xFF01
#define GATTS_DESCR_CHRONO_UUID     0x3333
#define GATTS_NUM_HANDLE_CHRONO     4
#define GATTS_SERVICE_PING_UUID     0x00EE
#define GATTS_CHAR_PING_PONG_UUID   0xFF02

/* Max Advertisename ESP_BLE_ADV_NAME_LEN_MAX = 29 chars*/

#define TEST_MANUFACTURER_DATA_LEN  17

#define GATTS_CHAR_VAL_LEN_MAX 0x40

#define PREPARE_BUF_MAX_SIZE 1024


static uint8_t char1_str[] = {0x00,0x00,0x00};
static esp_gatt_char_prop_t a_property = 0;

static esp_attr_value_t gatts_demo_char1_val = {
    .attr_max_len = GATTS_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(char1_str),
    .attr_value   = char1_str, // Ensure this is not NULL
};

static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)



static uint8_t adv_service_uuid128[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

// The length of adv data must be less than 31 bytes
//static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
//adv data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    //.min_interval = 0x0006,
    //.max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

#define PROFILE_NUM 1
#define PROFILE_CHRONO_ID 0

//CHRONO to CHRONO

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_CHRONO_ID] = {
        .gatts_cb = gatts_profile_a_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t a_prepare_write_env;

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

static void turn_on_led(uint8_t led_pin);

// Function to turn off an LED
static void turn_off_led(uint8_t led_pin);

#define NOTIFY_QUEUE_SIZE 10
static volatile QueueHandle_t notify_queue;

// Structure to hold notification data
typedef struct {
    uint32_t value;
} notify_data_t;

// GPIO pins for start and stop signals
#define GPIO_START_PIN 7
#define GPIO_STOP_PIN  6
#define GPIO_LED_GREEN 0
#define GPIO_LED_RED 1
//#define GPIO_UART_TXD 21
//#define GPIO_UART_RXD 20
static volatile int64_t timer_start_time = 0; // Variable to store the start time
static volatile bool timer_active = false;   // Flag to indicate if the timer is active
static void process_notify_queue(void);

// GPIO interrupt handler for start signal
//static void gpio_start_isr_handler(void *arg) {
//static void IRAM_ATTR gpio_start_isr_handler(void *arg) {
static void IRAM_ATTR gpio_start_isr_handler(void *arg) {
    //ESP_LOGE(GATTS_TAG, ">");
    if (!timer_active) {

        timer_start_time = esp_timer_get_time(); // Record the start time in microseconds
        timer_active = true;
    }
}

// GPIO interrupt handler for stop signal
// static void gpio_stop_isr_handler(void *arg) {
// static void IRAM_ATTR gpio_stop_isr_handler(void *arg) {
static void IRAM_ATTR gpio_stop_isr_handler(void *arg) {
    //ESP_LOGE(GATTS_TAG, ">");
    if (timer_active) {

        int64_t timer_stop_time = esp_timer_get_time(); // Record the stop time in microseconds
        int64_t elapsed_time = timer_stop_time - timer_start_time; // Calculate elapsed time        

        if (notify_queue != NULL) {
            // Send the elapsed time to the notify queue
            notify_data_t notify_data = { .value = (uint32_t)elapsed_time };
            turn_on_led(GPIO_LED_RED);
            if (xQueueSendFromISR(notify_queue, &notify_data, NULL) != pdPASS) {                
                //ESP_LOGE(GATTS_TAG, "Failed to send timer data to notify queue");
            }
        }
    }
    timer_active = false;
}

// Function to configure GPIO pins
static void configure_gpio_pins(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE, // Trigger on falling edge
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << GPIO_START_PIN) | (1ULL << GPIO_STOP_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);

    // Install GPIO ISR service
    gpio_install_isr_service(0);

    // Attach the interrupt handlers
    gpio_isr_handler_add(GPIO_START_PIN, gpio_start_isr_handler, NULL);
    gpio_isr_handler_add(GPIO_STOP_PIN, gpio_stop_isr_handler, NULL);
}

// Function to configure LED pins
static void configure_led_pins(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_LED_GREEN) | (1ULL << GPIO_LED_RED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // Turn off LEDs initially
    gpio_set_level(GPIO_LED_GREEN, 0);
    gpio_set_level(GPIO_LED_RED, 0);
}

// // Function to configure UART
// static void configure_uart(void) {
//     const uart_config_t uart_config = {
//         .baud_rate = 115200,
//         .data_bits = UART_DATA_8_BITS,
//         .parity = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//     };
//     uart_param_config(UART_NUM_0, &uart_config);
//     uart_set_pin(UART_NUM_0, GPIO_UART_TXD, GPIO_UART_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
//     uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);
// }

// static void write_debug_to_uart(const char *message) {
//     uart_write_bytes(UART_NUM_0, message, strlen(message));
// }

// Function to turn on an LED
static void turn_on_led(uint8_t led_pin) {
    gpio_set_level(led_pin, 1); // Set the GPIO level to HIGH
}

// Function to turn off an LED
static void turn_off_led(uint8_t led_pin) {
    gpio_set_level(led_pin, 0); // Set the GPIO level to LOW
}

// Modify update_notify_data_task to remove test code
static void update_notify_data_task(void *param) {
    while (1) {
        // Wait for notifications to be processed in the BLE thread
        process_notify_queue();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// Add a function to process the queue in the BLE thread
static void process_notify_queue(void) {
    if (notify_queue == NULL) {
        ESP_LOGE(GATTS_TAG, "Notify queue is not initialized");
        return;
    }
    
    notify_data_t notify_data;
    while (xQueueReceive(notify_queue, &notify_data, 0) == pdPASS) {
        ESP_LOGI(GATTS_TAG, "Process Q");
        uint8_t notify_value[4];
        notify_value[0] = (notify_data.value >> 24) & 0xFF;
        notify_value[1] = (notify_data.value >> 16) & 0xFF;
        notify_value[2] = (notify_data.value >> 8) & 0xFF;
        notify_value[3] = notify_data.value & 0xFF;
        ESP_LOGI("WS", "Send: %lu", notify_data.value);
        if (gl_profile_tab[PROFILE_CHRONO_ID].char_handle != 0) {
            esp_ble_gatts_set_attr_value(gl_profile_tab[PROFILE_CHRONO_ID].char_handle, sizeof(notify_value), notify_value);
        } else {
            ESP_LOGE(GATTS_TAG, "Characteristic handle is invalid");
        }

        // Send notification if enabled
        if (a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY) {
            esp_ble_gatts_send_indicate(gl_profile_tab[PROFILE_CHRONO_ID].gatts_if,
                                        gl_profile_tab[PROFILE_CHRONO_ID].conn_id,
                                        gl_profile_tab[PROFILE_CHRONO_ID].char_handle,
                                        sizeof(notify_value), notify_value, false);                             
        }
        turn_off_led(GPIO_LED_RED);  
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(GATTS_TAG, "Advertising data set complete");
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0) {
            esp_err_t adv_start_ret = esp_ble_gap_start_advertising(&adv_params);
            if (adv_start_ret) {
                ESP_LOGE(GATTS_TAG, "Failed to start advertising, error code = %x", adv_start_ret);
            } else {
                ESP_LOGI(GATTS_TAG, "Advertising started successfully");
            }
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising start failed, status %d", param->adv_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTS_TAG, "Advertising start successfully");
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising stop failed, status %d", param->adv_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTS_TAG, "Advertising stop successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(GATTS_TAG, "Connection params update, status %d, conn_int %d, latency %d, timeout %d",
                  param->update_conn_params.status,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
        ESP_LOGI(GATTS_TAG, "Packet length update, status %d, rx %d, tx %d",
                  param->pkt_data_length_cmpl.status,
                  param->pkt_data_length_cmpl.params.rx_len,
                  param->pkt_data_length_cmpl.params.tx_len);
        break;
    default:
        break;
    }
}

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp){
        if (param->write.is_prep) {
            if (param->write.offset > PREPARE_BUF_MAX_SIZE) {
                status = ESP_GATT_INVALID_OFFSET;
            } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
                status = ESP_GATT_INVALID_ATTR_LEN;
            }
            if (status == ESP_GATT_OK && prepare_write_env->prepare_buf == NULL) {
                prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE*sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;
                if (prepare_write_env->prepare_buf == NULL) {
                    ESP_LOGE(GATTS_TAG, "Gatt_server prep no mem");
                    status = ESP_GATT_NO_RESOURCES;
                }
            }

            esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
            if (gatt_rsp) {
                gatt_rsp->attr_value.len = param->write.len;
                gatt_rsp->attr_value.handle = param->write.handle;
                gatt_rsp->attr_value.offset = param->write.offset;
                gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
                memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
                esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
                if (response_err != ESP_OK){
                    ESP_LOGE(GATTS_TAG, "Send response error\n");
                }
                free(gatt_rsp);
            } else {
                ESP_LOGE(GATTS_TAG, "malloc failed, no resource to send response error\n");
                status = ESP_GATT_NO_RESOURCES;
            }
            if (status != ESP_GATT_OK){
                return;
            }
            memcpy(prepare_write_env->prepare_buf + param->write.offset,
                   param->write.value,
                   param->write.len);
            prepare_write_env->prepare_len += param->write.len;

        }else{
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC){
        ESP_LOG_BUFFER_HEX(GATTS_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }else{
        ESP_LOGI(GATTS_TAG,"Prepare write cancel");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TAG, "GATT server register, status %d, app_id %d, gatts_if %d", param->reg.status, param->reg.app_id, gatts_if);
        gl_profile_tab[PROFILE_CHRONO_ID].service_id.is_primary = true;
        gl_profile_tab[PROFILE_CHRONO_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[PROFILE_CHRONO_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_CHRONO_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_CHRONO_UUID;

		uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_BT);
        char device_name[20];
        snprintf(device_name, sizeof(device_name), "OpenChrono(%02X%02X)", mac[4], mac[5]);
        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(device_name);
        if (set_dev_name_ret){
            ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }
        //config adv data
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret){
            ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
        }
        adv_config_done |= adv_config_flag;
        //config scan response data
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret){
            ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
        }
        adv_config_done |= scan_rsp_config_flag;
        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_CHRONO_ID].service_id, GATTS_NUM_HANDLE_CHRONO);
        break;
    case ESP_GATTS_READ_EVT: {
        ESP_LOGI(GATTS_TAG, "Characteristic read, conn_id %d, trans_id %" PRIu32 ", handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);

        // Retrieve the current value of the characteristic
        uint16_t length = 0;
        const uint8_t *current_value = NULL;
        esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->read.handle, &length, &current_value);
        if (get_attr_ret == ESP_OK && current_value != NULL) {
            ESP_LOGI(GATTS_TAG, "Read current value, length = %d", length);
            ESP_LOG_BUFFER_HEX(GATTS_TAG, current_value, length);

            // Send the current value as the response
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.len = length;
            memcpy(rsp.attr_value.value, current_value, length);

            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        } else {
            ESP_LOGE(GATTS_TAG, "Failed to get current attribute value, error code = %x", get_attr_ret);
        }
        break;
    }
    case ESP_GATTS_WRITE_EVT: {
        ESP_LOGI(GATTS_TAG, "Characteristic write, conn_id %d, trans_id %" PRIu32 ", handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);

        if (!param->write.is_prep) {
            ESP_LOGI(GATTS_TAG, "value len %d, value ", param->write.len);
            ESP_LOG_BUFFER_HEX(GATTS_TAG, param->write.value, param->write.len);

            if (gl_profile_tab[PROFILE_CHRONO_ID].descr_handle == param->write.handle && param->write.len == 2){
                uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                if (descr_value == 0x0001){
                    if (a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY){
                        ESP_LOGI(GATTS_TAG, "Notification enable");
                        esp_ble_gatts_set_attr_value(gl_profile_tab[PROFILE_CHRONO_ID].descr_handle, sizeof(descr_value), (uint8_t *)&descr_value);

                        // Retrieve the current value of the characteristic
                        uint16_t length = 0;
                        const uint8_t *current_value = NULL;
                        esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(gl_profile_tab[PROFILE_CHRONO_ID].char_handle, &length, &current_value);
                        if (get_attr_ret == ESP_OK && current_value != NULL) {
                            esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_CHRONO_ID].char_handle,
                                                        length, (uint8_t *)current_value, false);
                        } else {
                            ESP_LOGE(GATTS_TAG, "Failed to get current attribute value, error code = %x", get_attr_ret);
                        }
                    }
                }else if (descr_value == 0x0002){
                    if (a_property & ESP_GATT_CHAR_PROP_BIT_INDICATE){
                        ESP_LOGI(GATTS_TAG, "Indication enable");
                        esp_ble_gatts_set_attr_value(gl_profile_tab[PROFILE_CHRONO_ID].descr_handle, sizeof(descr_value), (uint8_t *)&descr_value);
                        uint8_t indicate_data[15];
                        for (int i = 0; i < sizeof(indicate_data); ++i)
                        {
                            indicate_data[i] = i%0xff;
                        }
                        //the size of indicate_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_CHRONO_ID].char_handle,
                                                sizeof(indicate_data), indicate_data, true);
                    }
                }
                else if (descr_value == 0x0000){
                    ESP_LOGI(GATTS_TAG, "Notification/Indication disable");
                    esp_ble_gatts_set_attr_value(gl_profile_tab[PROFILE_CHRONO_ID].descr_handle, sizeof(descr_value), (uint8_t *)&descr_value);
                }else{
                    ESP_LOGE(GATTS_TAG, "Unknown descriptor value");
                    ESP_LOG_BUFFER_HEX(GATTS_TAG, param->write.value, param->write.len);
                }

            }
        }
        example_write_event_env(gatts_if, &a_prepare_write_env, param);
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
        ESP_LOGI(GATTS_TAG,"Execute write");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        example_exec_write_event_env(&a_prepare_write_env, param);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(GATTS_TAG, "MTU exchange completed, MTU size: %d", param->mtu.mtu);
        break;
    case ESP_GATTS_RESPONSE_EVT:
        ESP_LOGI(GATTS_TAG, "Response sent, status: %d, conn_id: %d, handle: %d",
                 param->rsp.status, param->rsp.conn_id, param->rsp.handle);
        break;
    case ESP_GATTS_SET_ATTR_VAL_EVT:
        ESP_LOGI(GATTS_TAG, "Set attribute value, status: %d, srvc_handle: %d, attr_handle: %d",
                 param->set_attr_val.status, param->set_attr_val.srvc_handle, param->set_attr_val.attr_handle);
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(GATTS_TAG, "Service create, status %d, service_handle %d", param->create.status, param->create.service_handle);
        if (param->create.status != ESP_GATT_OK) {
            ESP_LOGE(GATTS_TAG, "Failed to create service, status = %d", param->create.status);
            break;
        }
        gl_profile_tab[PROFILE_CHRONO_ID].service_handle = param->create.service_handle;

        // Add the first characteristic (0xFF01)
        gl_profile_tab[PROFILE_CHRONO_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_CHRONO_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_CHRONO_UUID;
        a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[PROFILE_CHRONO_ID].service_handle,
                                                        &gl_profile_tab[PROFILE_CHRONO_ID].char_uuid,
                                                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                        a_property,
                                                        &gatts_demo_char1_val, NULL);
        if (add_char_ret) {
            ESP_LOGE(GATTS_TAG, "Failed to add characteristic 0xFF01, error code = %x", add_char_ret);
        } else {
            ESP_LOGI(GATTS_TAG, "Characteristic 0xFF01 added successfully");
        }

        // Start the service
        esp_err_t start_service_ret = esp_ble_gatts_start_service(gl_profile_tab[PROFILE_CHRONO_ID].service_handle);
        if (start_service_ret) {
            ESP_LOGE(GATTS_TAG, "Failed to start service, error code = %x", start_service_ret);
        } else {
            ESP_LOGI(GATTS_TAG, "Service started successfully");
        }
        break;
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;
    case ESP_GATTS_ADD_CHAR_EVT: {
        uint16_t length = 0;
        const uint8_t *prf_char;

        ESP_LOGI(GATTS_TAG, "Characteristic add, status %d, attr_handle %d, service_handle %d",
                param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
        gl_profile_tab[PROFILE_CHRONO_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[PROFILE_CHRONO_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_CHRONO_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle,  &length, &prf_char);
        if (get_attr_ret == ESP_FAIL){
            ESP_LOGE(GATTS_TAG, "ILLEGAL HANDLE");
        }

        ESP_LOGI(GATTS_TAG, "the gatts demo char length = %x", length);
        for(int i = 0; i < length; i++){
            ESP_LOGI(GATTS_TAG, "prf_char[%x] =%x",i,prf_char[i]);
        }
        esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_CHRONO_ID].service_handle, &gl_profile_tab[PROFILE_CHRONO_ID].descr_uuid,
                                                                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        if (add_descr_ret){
            ESP_LOGE(GATTS_TAG, "add char descr failed, error code =%x", add_descr_ret);
        }
        break;
    }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        gl_profile_tab[PROFILE_CHRONO_ID].descr_handle = param->add_char_descr.attr_handle;
        ESP_LOGI(GATTS_TAG, "Descriptor add, status %d, attr_handle %d, service_handle %d",
                 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TAG, "Service start, status %d, service_handle %d",
                 param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT: {
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
        turn_on_led(GPIO_LED_GREEN);
        ESP_LOGI(GATTS_TAG, "Connected, conn_id %u, remote "ESP_BD_ADDR_STR"",
                 param->connect.conn_id, ESP_BD_ADDR_HEX(param->connect.remote_bda));
        gl_profile_tab[PROFILE_CHRONO_ID].conn_id = param->connect.conn_id;
        //start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT:
        turn_off_led(GPIO_LED_GREEN);
        ESP_LOGI(GATTS_TAG, "Disconnected, remote "ESP_BD_ADDR_STR", reason 0x%02x",
                 ESP_BD_ADDR_HEX(param->disconnect.remote_bda), param->disconnect.reason);
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(GATTS_TAG, "Confirm received, status %d, attr_handle %d", param->conf.status, param->conf.handle);
        break;
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
        ESP_LOGW(GATTS_TAG, "Unhandled event: %d", event);
        break;
    }
    // Only process the notify queue if it is initialized
    if (notify_queue != NULL) {
        process_notify_queue();
    } else {
        ESP_LOGW(GATTS_TAG, "Notify queue is not initialized, skipping queue processing");
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* Call Profile A callback */
    for (int idx = 0; idx < PROFILE_NUM; idx++) {
        if (gatts_if == ESP_GATT_IF_NONE || gatts_if == gl_profile_tab[idx].gatts_if) {
            if (gl_profile_tab[idx].gatts_cb) {
                gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
            }
        }
    }
}

void app_main(void)
{
    esp_err_t ret;
    ESP_LOGI("DEBUG", "Reached app_main");
    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(PROFILE_CHRONO_ID);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return;
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    // Create the notify queue
    notify_queue = xQueueCreate(NOTIFY_QUEUE_SIZE, sizeof(notify_data_t));
    if (notify_queue == NULL) {
        ESP_LOGE(GATTS_TAG, "Failed to create notify queue");
        return;
    }

// Configure GPIO pins for LEDs and UART
configure_led_pins();

// Configure GPIO pins for start and stop signals
configure_gpio_pins();

turn_on_led(GPIO_LED_GREEN);
turn_off_led(GPIO_LED_GREEN);
turn_on_led(GPIO_LED_RED);
turn_off_led(GPIO_LED_RED);

xTaskCreate(update_notify_data_task, "update_notify_data_task", 2048, NULL, 5, NULL);
ESP_LOGI("WS", "APP_MAIN_DONE");
}
