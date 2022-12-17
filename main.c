
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "bmp180.h"
#include "rom/ets_sys.h"

#include "driver/adc.h"
#include "driver/ledc.h"
#include "pwm_control.h"

static const char *BMPTAG = "BMP";

static const char *TAG = "MQTT_EXAMPLE";
#define EXAMPLE_ESP_WIFI_SSID "Uilatech2"
#define EXAMPLE_ESP_WIFI_PASS "Uilatech*123"

uint32_t MQTT_CONNEECTED = 0;

static void mqtt_app_start(void);

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id)
    {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        ESP_LOGI(TAG, "Trying to connect with Wi-Fi\n");
        break;

    case SYSTEM_EVENT_STA_CONNECTED:
        ESP_LOGI(TAG, "Wi-Fi connected\n");
        break;

    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "got ip: startibg MQTT Client\n");
        mqtt_app_start();
        break;

    case SYSTEM_EVENT_STA_DISCONNECTED:
        ESP_LOGI(TAG, "disconnected: Retrying Wi-Fi\n");
        esp_wifi_connect();
        break;

    default:
        break;
    }
    return ESP_OK;
}

void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
char pwm_data[100];
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        MQTT_CONNEECTED = 1;

        msg_id = esp_mqtt_client_subscribe(client, "/topic/test1", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/test2", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        MQTT_CONNEECTED = 0;
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        sprintf(pwm_data, "%.*s", event-> data_len,event->data);

        int pwmval = (atoi(pwm_data));
        printf("pwmmmvaaal:%d", pwmval);
        motor_drive();
        ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, pwmval);
        ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);

        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

esp_mqtt_client_handle_t client = NULL;
static void mqtt_app_start(void)
{
    ESP_LOGI(TAG, "STARTING MQTT");
    esp_mqtt_client_config_t mqttConfig = {
        .uri = "mqtt://192.168.29.170:1883"};

    client = esp_mqtt_client_init(&mqttConfig);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

float temp;
int32_t press;

void start_bmp180_default()
{

    while (1)
    {

        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Connect to BMP180 using i2c settings.
 * In case if deafult setting like clk_speed, sda_io and etc. need to be changed.
 */
void start_bmp180_i2c()
{
    ESP_LOGI(BMPTAG, "init: start_bmp180_i2c");
    bmp_sensor_t *sensor;
    tuple_t *pair;

    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 18, // SDA_PIN,
        .scl_io_num = 19, // SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 1000,
    };

    sensor = create_bmp_sensor_i2c(i2c_config, BMP085_MODE_STANDARD);

    if (sensor == NULL)
    {
        ESP_LOGE(BMPTAG, "cannot create/init sensor");
        return;
    }

    sensor->begin(sensor);

    /* Get tuple with temp and pressure. Tuple needs to be deleted. */
    pair = sensor->get_tuple(sensor);
    if (pair)
    {
        ESP_LOGI(BMPTAG, "Temp: %f, Pressure: %d", pair->temperature, pair->pressure);
        free(pair);
    }

    sensor->destroy(sensor);
}

/**
 * @brief Connect to BMP180 using gpios.
 */
void start_bmp180_gpio()
{
    ESP_LOGI(BMPTAG, "init: start_bmp180_gpio");
    bmp_sensor_t *sensor;
    tuple_t *pair;

    sensor = create_bmp_sensor(18, 19, BMP085_MODE_STANDARD);

    if (sensor == NULL)
    {
        ESP_LOGE(BMPTAG, "cannot create/init sensor");
        return;
    }

    sensor->begin(sensor);

    /* Get tuple with temp and pressure. Tuple needs to be deleted. */
    pair = sensor->get_tuple(sensor);
    if (pair)
    {
        ESP_LOGI(BMPTAG, "Temp: %f, Pressure: %d", pair->temperature, pair->pressure);
        free(pair);
    }

    sensor->destroy(sensor);
}

void Publisher_Task(void *params)
{
    ESP_LOGI(BMPTAG, "init: start_bmp180_default");
    bmp_sensor_t *sensor = create_bmp_sensor_default();

    if (sensor == NULL)
    {
        ESP_LOGE(BMPTAG, "cannot create/init sensor");
        return;
    }

    sensor->begin(sensor);
    while (true)
    {
        if (MQTT_CONNEECTED)
        {
            temp = sensor->get_temperature(sensor);
            press = sensor->get_pressure(sensor);

            ESP_LOGI(BMPTAG, "Temp: %f, Pressure: %d", temp, press);

            // strcpy(str, temp);
            // printf("Temperature: %.2fC\n" , temp);
            printf(pwm_data);
            ESP_LOGI("PWM", "pwmdata : %s", pwm_data);
            char temperature[12];
            sprintf(temperature, "%.2f", temp);
            esp_mqtt_client_publish(client, "$device/shadow/device1/update", temperature, 0, 0, 0);
            vTaskDelay(15000 / portTICK_PERIOD_MS);
        }
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    init_hw();
    wifi_init();
    start_bmp180_i2c();
    start_bmp180_gpio();

    xTaskCreate(Publisher_Task, "Publisher_Task", 1024 * 5, NULL, 5, NULL);
}