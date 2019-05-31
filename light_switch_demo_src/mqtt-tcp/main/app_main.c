#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "driver/uart.h"
#include "app_prov.h"

#define SW1_BUTTON_GPIO 33

static const char *TAG = "LED-MESH-GW";

static EventGroupHandle_t wifi_event_group;
const static int CONNECTED_BIT = BIT0;

uint8_t m_mac[6];
#define TOPIC_LEN 128
char m_pub_topic[TOPIC_LEN];
char m_sub_topic[TOPIC_LEN];
char m_sub_topic_recover[TOPIC_LEN];

#define UART1_TXD  (GPIO_NUM_5)
#define UART1_RXD  (GPIO_NUM_18)
#define UART1_RTS  (UART_PIN_NO_CHANGE)
#define UART1_CTS  (UART_PIN_NO_CHANGE)

#define MESH_CLIENT_RESET (GPIO_NUM_15)

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

#define EX_UART_NUM UART_NUM_1
static QueueHandle_t uart_queue;


esp_mqtt_client_handle_t m_pub_client=-1;

static void gpio_set(gpio_num_t pin, uint32_t level)
{
    /* Configure the button GPIO as input, enable wakeup */
    const int button_gpio_num = pin;
    gpio_config_t config = {
            .pin_bit_mask = BIT64(button_gpio_num),
            .mode = GPIO_MODE_OUTPUT
    };
    ESP_ERROR_CHECK(gpio_config(&config));

    gpio_set_level(pin, level);
}

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);

            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    //ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                    ESP_LOG_BUFFER_HEXDUMP(TAG, dtmp, event.size, ESP_LOG_INFO);
                    if(m_pub_client != -1)
                    {
                        int msg_id = esp_mqtt_client_publish(m_pub_client, m_pub_topic, (const char*)dtmp, 0/*event.size*/, 0, 0);
                        //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
                    }
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

static void uart_init(void)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(EX_UART_NUM, &uart_config);

    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, UART1_TXD, UART1_RXD, UART1_RTS, UART1_CTS);
    //Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart_queue, 0);

    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
    
}


static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            m_pub_client = client;
            
            msg_id = esp_mqtt_client_subscribe(client, m_sub_topic, 0);
            msg_id = esp_mqtt_client_subscribe(client, m_sub_topic_recover, 0);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
            
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            #if 0
            msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            #endif
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
            // Write data back to the UART
            if(!strncmp(event->topic, m_sub_topic, event->topic_len))
            {
                uart_write_bytes(EX_UART_NUM, (const char *) event->data, event->data_len);
                uart_write_bytes(EX_UART_NUM, (const char *) "\n", 1);
            } else if(!strncmp(event->topic, m_sub_topic_recover, event->topic_len))
            {
                gpio_set(MESH_CLIENT_RESET, 0);
                vTaskDelay(100 / portTICK_PERIOD_MS);
                gpio_set(MESH_CLIENT_RESET, 1);
            }
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URL,
        .event_handle = mqtt_event_handler,
        // .user_context = (void *)your_context
    };

#if CONFIG_BROKER_URL_FROM_STDIN
    char line[128];

    if (strcmp(mqtt_cfg.uri, "FROM_STDIN") == 0) {
        int count = 0;
        printf("Please enter url of mqtt broker\n");
        while (count < 128) {
            int c = fgetc(stdin);
            if (c == '\n') {
                line[count] = '\0';
                break;
            } else if (c > 0 && c < 127) {
                line[count] = c;
                ++count;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        mqtt_cfg.uri = line;
        printf("Broker url: %s\n", line);
    } else {
        ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
        abort();
    }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    /* Invoke Provisioning event handler first */
    app_prov_event_handler(ctx, event);

    switch(event->event_id) {
    case SYSTEM_EVENT_AP_START:
        ESP_LOGI(TAG, "SoftAP started");
        break;
    case SYSTEM_EVENT_AP_STOP:
        ESP_LOGI(TAG, "SoftAP stopped");
        break;
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "got ip:%s",
                 ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        mqtt_app_start();
        break;
    case SYSTEM_EVENT_AP_STACONNECTED:
        ESP_LOGI(TAG, "station:"MACSTR" join, AID=%d",
                 MAC2STR(event->event_info.sta_connected.mac),
                 event->event_info.sta_connected.aid);
        break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
        ESP_LOGI(TAG, "station:"MACSTR"leave, AID=%d",
                 MAC2STR(event->event_info.sta_disconnected.mac),
                 event->event_info.sta_disconnected.aid);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
	    esp_wifi_connect();
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void wifi_init_sta()
{
    /* Start wifi in station mode with credentials set during provisioning */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_start() );
}

static bool is_SW1_pressed()
{
    int level;
    /* Configure the button GPIO as input, enable wakeup */
    const int button_gpio_num = SW1_BUTTON_GPIO;
    gpio_config_t config = {
            .pin_bit_mask = BIT64(button_gpio_num),
            .mode = GPIO_MODE_INPUT
    };
    ESP_ERROR_CHECK(gpio_config(&config));
    
    level = gpio_get_level(SW1_BUTTON_GPIO);
    ESP_LOGI(TAG, "level: %d", level);
    return (level ? false : true);
}

static void wifi_provision()
{
    /* Security version */
    int security = 0;
    /* Proof of possession */
    const protocomm_security_pop_t *pop = NULL;

#ifdef CONFIG_USE_SEC_1
    security = 1;
#endif

    /* Having proof of possession is optional */
#ifdef CONFIG_USE_POP
    const static protocomm_security_pop_t app_pop = {
        .data = (uint8_t *) CONFIG_POP,
        .len = (sizeof(CONFIG_POP)-1)
    };
    pop = &app_pop;
#endif

    /* Initialize networking stack */
    tcpip_adapter_init();

    /* Set our event handling */
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    /* Check if device is provisioned */
    bool provisioned;
    if (app_prov_is_provisioned(&provisioned) != ESP_OK) {
        ESP_LOGE(TAG, "Error getting device provisioning state");
        return;
    }

    if(is_SW1_pressed())
    {
        ESP_LOGI(TAG, "Force to re-provision");
        provisioned = false;
    }

    if (provisioned == false) {
        /* If not provisioned, start provisioning via soft AP */
        ESP_LOGI(TAG, "Starting WiFi SoftAP provisioning");

        const char *ssid = NULL;

#ifdef CONFIG_SOFTAP_SSID
        ssid = CONFIG_SOFTAP_SSID;
#else
        uint8_t eth_mac[6];
        esp_wifi_get_mac(WIFI_IF_STA, eth_mac);

        char ssid_with_mac[33];
        snprintf(ssid_with_mac, sizeof(ssid_with_mac), "PROV_%02X%02X%02X",
                 eth_mac[3], eth_mac[4], eth_mac[5]);

        ssid = ssid_with_mac;
#endif

        app_prov_start_softap_provisioning(ssid, CONFIG_SOFTAP_PASS,
                                           security, pop);
    } else {
        /* Start WiFi station with credentials set during provisioning */
        ESP_LOGI(TAG, "Starting WiFi station");
        wifi_init_sta(NULL);
    }
}

void app_main()
{    
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());
    
    esp_efuse_mac_get_default(m_mac);
    ESP_LOGI(TAG, "[APP] MAC: %X:%X:%X:%X:%X:%X", m_mac[0], m_mac[1], m_mac[2], m_mac[3], m_mac[4], m_mac[5]);

    memset(m_sub_topic, 0, TOPIC_LEN * sizeof(char));
    memset(m_sub_topic_recover, 0, TOPIC_LEN * sizeof(char));
    memset(m_pub_topic, 0, TOPIC_LEN * sizeof(char));
    sprintf(m_pub_topic, "devices/LED-GW/LED-GW-0001000%02X%02X%02X/telemetry", m_mac[3], m_mac[4], m_mac[5]);
    sprintf(m_sub_topic, "devices/LED-GW/LED-GW-0001000%02X%02X%02X/config", m_mac[3], m_mac[4], m_mac[5]);
    sprintf(m_sub_topic_recover, "devices/LED-GW/LED-GW-0001000%02X%02X%02X/recover_mesh_client", m_mac[3], m_mac[4], m_mac[5]);
    
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    wifi_provision();
    
    uart_init();
    
}
