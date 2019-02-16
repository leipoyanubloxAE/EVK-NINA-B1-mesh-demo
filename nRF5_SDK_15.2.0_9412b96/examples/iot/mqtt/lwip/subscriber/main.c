/**
 * Copyright (c) 2013 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 *
 * @defgroup iot_sdk_app_mqtt_client main.c
 * @{
 * @ingroup iot_sdk_app_lwip
 *
 * @brief This file contains the source code for LwIP based MQTT Client sample application.
 *        This example subscribes for topic "led/state" and based on the state 0 or 1
 *        turns ON or OFF its own led.
 */

//#define MQTT_TEST 1

#define MESH_TEST 1

#ifdef MESH_TEST
#include <stdint.h>
#include <string.h>

/* HAL */
#include "boards.h"
#include "simple_hal.h"
#include "app_timer.h"

/* Core */
#include "nrf_mesh_config_core.h"
#include "nrf_mesh_gatt.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "device_state_manager.h"
#include "access_config.h"

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

/* Models */
#include "generic_onoff_client.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Example specific includes */
#include "app_config.h"
#include "nrf_mesh_config_examples.h"
#include "light_switch_example_common.h"
#include "example_common.h"
#include "ble_softdevice_support.h"
#endif

#ifdef MQTT_TEST
#include <stdbool.h>
#include <stdint.h>
#include "boards.h"
#include "nordic_common.h"
#include "app_scheduler.h"
#include "sdk_config.h"
#include "app_timer.h"
#include "app_button.h"
#include "lwip/init.h"
#include "lwip/inet6.h"
#include "lwip/ip6.h"
#include "lwip/ip6_addr.h"
#include "lwip/netif.h"
#include "mqtt.h"
#include "lwip/timers.h"
#include "nrf_platform_port.h"
#include "app_util_platform.h"
#include "iot_timer.h"
#include "ipv6_medium.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#if 0
#include "nrf_sdh_soc.h"

#define MESH_SOC_OBSERVER_PRIO 0

static void mesh_soc_evt_handler(uint32_t evt_id, void * p_context)
{
    nrf_mesh_on_sd_evt(evt_id);
}
NRF_SDH_SOC_OBSERVER(m_mesh_soc_observer, MESH_SOC_OBSERVER_PRIO, mesh_soc_evt_handler, NULL);
#endif

#define EVK-NINA-B1 1

/** Modify m_broker_addr according to your setup.
 *  The address provided below is a place holder.  */
static const ipv6_addr_t m_broker_addr =
{
    .u8 =
    {0x20, 0x01, 0x41, 0xd0,
     0x00, 0x0a, 0x3a, 0x10,
     0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x01}
};

#define LED_ONE                             BSP_LED_0_MASK
#define LED_TWO                             BSP_LED_1_MASK
#define LED_THREE                           BSP_LED_2_MASK
#define ALL_APP_LED                        (BSP_LED_0_MASK | BSP_LED_1_MASK | \
                                            BSP_LED_2_MASK )                        /**< Define used for simultaneous operation of all application LEDs. */


#define LWIP_SYS_TICK_MS                    10                                                      /**< Interval for timer used as trigger to send. */
#define LED_BLINK_INTERVAL_MS               300                                                     /**< LED blinking interval. */

#define SCHED_MAX_EVENT_DATA_SIZE           16                                                      /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE                    192                                                     /**< Maximum number of events in the scheduler queue. */

#define DEAD_BEEF                           0xDEADBEEF                                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APP_ENABLE_LOGS                     1                                                       /**< Enable logs in the application. */

#define APP_MQTT_BROKER_PORT                1883                                                    /**< Port number of MQTT Broker being used. */
#define APP_MQTT_SUBSCRIPTION_PKT_ID        10                                                      /**< Unique identification of subscription, can be any unsigned 16 bit integer value. */
#define APP_MQTT_SUBSCRIPTION_TOPIC         "led/state"                                             /**< MQTT topic to which this application subscribes. */

APP_TIMER_DEF(m_iot_timer_tick_src_id);                                                             /**< System Timer used to service CoAP and LWIP periodically. */
eui64_t                                     eui64_local_iid;                                        /**< Local EUI64 value that is used as the IID for*/
static ipv6_medium_instance_t               m_ipv6_medium;
static mqtt_client_t                        m_app_mqtt_client;                                      /**< MQTT Client instance reference provided by the MQTT module. */
static const char                           m_client_id[] = "nrfSubscriber";                        /**< Unique MQTT client identifier. */

void app_mqtt_evt_handler(mqtt_client_t * const p_client, const mqtt_evt_t * p_evt);


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by this application.
 */
static void leds_init(void)
{
    // Configure application LED pins.
    LEDS_CONFIGURE(ALL_APP_LED);

    // Turn off all LED on initialization.
    LEDS_OFF(ALL_APP_LED);
}

/**@brief Timer callback used for controlling board LEDs to represent application state.
 *
 * @param[in]   wall_clock_value   The value of the wall clock that triggered the callback.
 */
static void blink_timeout_handler(iot_timer_time_in_ms_t wall_clock_value)
{
}

/**@brief Connect to MQTT broker. */
static void app_mqtt_connect(void)
{
    mqtt_client_init(&m_app_mqtt_client);

    memcpy(m_app_mqtt_client.broker_addr.u8, m_broker_addr.u8, IPV6_ADDR_SIZE);
    m_app_mqtt_client.broker_port          = APP_MQTT_BROKER_PORT;
    m_app_mqtt_client.evt_cb               = app_mqtt_evt_handler;
    m_app_mqtt_client.client_id.p_utf_str  = (uint8_t *)m_client_id;
    m_app_mqtt_client.client_id.utf_strlen = strlen(m_client_id);
    m_app_mqtt_client.p_password           = NULL;
    m_app_mqtt_client.p_user_name          = NULL;
    m_app_mqtt_client.transport_type       = MQTT_TRANSPORT_NON_SECURE;
    m_app_mqtt_client.p_security_settings  = NULL;

    UNUSED_VARIABLE(mqtt_connect(&m_app_mqtt_client));
}


/**@brief Subscribe with the broker. */
static void app_mqtt_subscribe(void)
{
    const char * topic_str = APP_MQTT_SUBSCRIPTION_TOPIC;

    mqtt_topic_t topic =
    {
        .topic =
        {
            .p_utf_str  = (uint8_t *)topic_str,
            .utf_strlen = strlen(topic_str)
        },
        .qos = MQTT_QoS_1_ATLEAST_ONCE
    };

    const mqtt_subscription_list_t subscription_list =
    {
        .p_list     = &topic,
        .list_count = 1,
        .message_id = APP_MQTT_SUBSCRIPTION_PKT_ID
    };

    uint32_t err_code = mqtt_subscribe(&m_app_mqtt_client, &subscription_list);

    if (err_code == NRF_SUCCESS)
    {
    }
}


/**@brief Unsubscribe with the broker. */
static void app_mqtt_unsubscribe(void)
{
    const char * topic_str = APP_MQTT_SUBSCRIPTION_TOPIC;

    mqtt_topic_t topic =
    {
        .topic =
        {
            .p_utf_str  = (uint8_t *)topic_str,
            .utf_strlen = strlen(topic_str)
        },
        .qos = MQTT_QoS_0_AT_MOST_ONCE
    };

    const mqtt_subscription_list_t subscription_list =
    {
        .p_list     = &topic,
        .list_count = 1,
        .message_id = APP_MQTT_SUBSCRIPTION_PKT_ID
    };

    uint32_t err_code = mqtt_unsubscribe(&m_app_mqtt_client, &subscription_list);

    if (err_code == NRF_SUCCESS)
    {
    }
}


static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    if (button_action == APP_BUTTON_PUSH)
    {
        switch (pin_no)
        {
            case BSP_BUTTON_0:
            {
                NRF_LOG_INFO ("app_mqtt_connect...");
                app_mqtt_connect();
                break;
            }
            default:
                break;
        }
    }
}

static void button_init(void)
{
    uint32_t err_code;

    static app_button_cfg_t buttons[] =
    {
        {BSP_BUTTON_0,        false, BUTTON_PULL, button_event_handler},
        {BSP_BUTTON_1,        false, BUTTON_PULL, button_event_handler},
    };

    #define BUTTON_DETECTION_DELAY APP_TIMER_TICKS(50)

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons), BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);

    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing IP stack.
 *
 * @details Initialize the IP Stack and its driver.
 */
static void ip_stack_init(void)
{
    uint32_t err_code;

    err_code = ipv6_medium_eui64_get(m_ipv6_medium.ipv6_medium_instance_id,
                                     &eui64_local_iid);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_mem_init();
    APP_ERROR_CHECK(err_code);

    // Initialize LwIP stack.
    lwip_init();

    // Initialize LwIP stack driver.
    err_code = nrf_driver_init();
    APP_ERROR_CHECK(err_code);

    err_code = mqtt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Timer callback used for periodic servicing of LwIP protocol timers.
 *        This trigger is also used in the example to trigger sending TCP Connection.
 *
 * @details Timer callback used for periodic servicing of LwIP protocol timers.
 *
 * @param[in]   wall_clock_value   The value of the wall clock that triggered the callback.
 */
static void system_timer_callback(iot_timer_time_in_ms_t wall_clock_value)
{
    UNUSED_VARIABLE(wall_clock_value);

    sys_check_timeouts();
    UNUSED_VARIABLE(mqtt_live());
}


/**@brief Function for updating the wall clock of the IoT Timer module.
 *
 * @param[in]   p_context   Pointer used for passing context. No context used in this application.
 */
static void iot_timer_tick_callback(void * p_context)
{
    UNUSED_VARIABLE(p_context);

    uint32_t err_code = iot_timer_update();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_ERROR_CHECK(app_timer_init());

    // Create a sys timer.
    err_code = app_timer_create(&m_iot_timer_tick_src_id,
                                APP_TIMER_MODE_REPEATED,
                                iot_timer_tick_callback);

    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the IoT Timer. */
static void iot_timer_init(void)
{
    uint32_t err_code;
#if 1
    static const iot_timer_client_t list_of_clients[] =
    {
        {system_timer_callback,      LWIP_SYS_TICK_MS},
     /*   {blink_timeout_handler,      LED_BLINK_INTERVAL_MS},*/
    };

    // The list of IoT Timer clients is declared as a constant.
    static const iot_timer_clients_list_t iot_timer_clients =
    {
        (sizeof(list_of_clients) / sizeof(iot_timer_client_t)),
        &(list_of_clients[0]),
    };

    // Passing the list of clients to the IoT Timer module.
    err_code = iot_timer_client_list_set(&iot_timer_clients);
    APP_ERROR_CHECK(err_code);
#endif
    // Starting the app timer instance that is the tick source for the IoT Timer.
    err_code = app_timer_start(m_iot_timer_tick_src_id,
                               APP_TIMER_TICKS(IOT_TIMER_RESOLUTION_IN_MS),
                               NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

/**@brief Function to handle interface up event. */
void nrf_driver_interface_up(iot_interface_t const * p_interface)
{
    UNUSED_PARAMETER(p_interface);

    NRF_LOG_INFO ("IPv6 Interface Up.");
    LEDS_ON(LED_ONE);

    sys_check_timeouts();
}


/**@brief Function to handle interface down event. */
void nrf_driver_interface_down(iot_interface_t const * p_interface)
{
    UNUSED_PARAMETER(p_interface);

    NRF_LOG_INFO ("IPv6 Interface Down.");
}

void app_mqtt_evt_handler(mqtt_client_t * const p_client, const mqtt_evt_t * p_evt)
{
    switch (p_evt->id)
    {
        case MQTT_EVT_CONNACK:
        {
            NRF_LOG_INFO (">> MQTT_EVT_CONNACK");
            if (p_evt->result == NRF_SUCCESS)
            {
                app_mqtt_subscribe();
                NRF_LOG_INFO (">> mqtt_subscribe");
                LEDS_OFF(LED_ONE);
                LEDS_ON(LED_TWO);
            }
            break;
        }
        case MQTT_EVT_PUBLISH:
        {
            NRF_LOG_INFO (">> Data length 0x%04lX", p_evt->param.publish.message.payload.bin_strlen);
            NRF_LOG_INFO (">> Topic length 0x%04lX", p_evt->param.publish.message.topic.topic.utf_strlen);

            if (p_evt->param.publish.message.payload.bin_strlen == 1)
            {
                // Accept binary or ASCII 0 and 1.
                if ((p_evt->param.publish.message.payload.p_bin_str[0] == 0) ||
                    (p_evt->param.publish.message.payload.p_bin_str[0] == 0x30))
                {
                    LEDS_OFF(LED_ONE);
                }
                else if ((p_evt->param.publish.message.payload.p_bin_str[0] == 1) ||
                         (p_evt->param.publish.message.payload.p_bin_str[0] == 0x31))
                {
                    LEDS_ON(LED_ONE);
                }
            }
            if (p_evt->param.publish.message.topic.qos == MQTT_QoS_1_ATLEAST_ONCE)
            {
                const mqtt_puback_param_t ack_param = {
                    .message_id = p_evt->param.publish.message_id
                };

                // Send acknowledgment.
                uint32_t err_code = mqtt_publish_ack(p_client, &ack_param);

                NRF_LOG_INFO (">> mqtt_publish_ack message id 0x%04x, result 0x%08lx",
                          p_evt->param.publish.message_id,
                          err_code);
                UNUSED_VARIABLE(err_code);
            }

            break;
        }
        case MQTT_EVT_DISCONNECT:
        {
            NRF_LOG_INFO (">> MQTT_EVT_DISCONNECT");
            break;
        }
        default:
            break;
    }
}


/**@brief Function for starting connectable mode.
 */
static void connectable_mode_enter(void)
{
    uint32_t err_code = ipv6_medium_connectable_mode_enter(m_ipv6_medium.ipv6_medium_instance_id);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Physical layer in connectable mode.");

}


static void on_ipv6_medium_evt(ipv6_medium_evt_t * p_ipv6_medium_evt)
{

}


static void on_ipv6_medium_error(ipv6_medium_error_t * p_ipv6_medium_error)
{
    // Do something.
}

/**@brief Function for initializing the nrf log module.
 */
static void sdk_log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}
#endif

#ifdef MESH_TEST
/**
 *------------------------------------------
 * Start of mesh app 
 *------------------------------------------
 */

#define APP_STATE_OFF                (0)
#define APP_STATE_ON                 (1)

#define APP_UNACK_MSG_REPEAT_COUNT   (2)

static generic_onoff_client_t m_clients[CLIENT_MODEL_INSTANCE_COUNT];
static bool                   m_device_provisioned;

/* Forward declaration */
static void app_gen_onoff_client_publish_interval_cb(access_model_handle_t handle, void * p_self);
static void app_generic_onoff_client_status_cb(const generic_onoff_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const generic_onoff_status_params_t * p_in);
static void app_gen_onoff_client_transaction_status_cb(access_model_handle_t model_handle,
                                                       void * p_args,
                                                       access_reliable_status_t status);

const generic_onoff_client_callbacks_t client_cbs =
{
    .onoff_status_cb = app_generic_onoff_client_status_cb,
    .ack_transaction_status_cb = app_gen_onoff_client_transaction_status_cb,
    .periodic_publish_cb = app_gen_onoff_client_publish_interval_cb
};

static void device_identification_start_cb(uint8_t attention_duration_s)
{
    hal_led_mask_set(LEDS_MASK, false);
    hal_led_blink_ms(BSP_LED_2_MASK  | BSP_LED_3_MASK,
                     LED_BLINK_ATTENTION_INTERVAL_MS,
                     LED_BLINK_ATTENTION_COUNT(attention_duration_s));
}

static void provisioning_aborted_cb(void)
{
    hal_led_blink_stop();
}

static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");

#if MESH_FEATURE_GATT_ENABLED
    /* Restores the application parameters after switching from the Provisioning
     * service to the Proxy  */
    gap_params_init();
    conn_params_init();
#endif

    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);

    hal_led_blink_stop();
    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
}

/* This callback is called periodically if model is configured for periodic publishing */
static void app_gen_onoff_client_publish_interval_cb(access_model_handle_t handle, void * p_self)
{
     __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Publish desired message here.\n");
}

/* Acknowledged transaction status callback, if acknowledged transfer fails, application can
* determine suitable course of action (e.g. re-initiate previous transaction) by using this
* callback.
*/
static void app_gen_onoff_client_transaction_status_cb(access_model_handle_t model_handle,
                                                       void * p_args,
                                                       access_reliable_status_t status)
{
    switch(status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer success.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
            hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer timeout.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_CANCELLED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer cancelled.\n");
            break;

        default:
            ERROR_CHECK(NRF_ERROR_INTERNAL);
            break;
    }
}

/* Generic OnOff client model interface: Process the received status message in this callback */
static void app_generic_onoff_client_status_cb(const generic_onoff_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const generic_onoff_status_params_t * p_in)
{
    if (p_in->remaining_time_ms > 0)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "OnOff server: 0x%04x, Present OnOff: %d, Target OnOff: %d, Remaining Time: %d ms\n",
              p_meta->src.value, p_in->present_on_off, p_in->target_on_off, p_in->remaining_time_ms);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "OnOff server: 0x%04x, Present OnOff: %d\n",
              p_meta->src.value, p_in->present_on_off);
    }
}

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);
    /* This function may return if there are ongoing flash operations. */
    mesh_stack_device_reset();
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
}

static void mesh_button_event_handler(uint32_t button_number)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);

    uint32_t status = NRF_SUCCESS;
    generic_onoff_set_params_t set_params;
    model_transition_t transition_params;
    static uint8_t tid = 0;

    /* Button 1: On, Button 2: Off, Client[0]
     * Button 2: On, Button 3: Off, Client[1]
     */

    switch(button_number)
    {
        case 0:
        case 2:
            set_params.on_off = APP_STATE_ON;
            break;

        case 1:
        case 3:
            set_params.on_off = APP_STATE_OFF;
            break;
    }

    set_params.tid = tid++;
    transition_params.delay_ms = APP_CONFIG_ONOFF_DELAY_MS;
    transition_params.transition_time_ms = APP_CONFIG_ONOFF_TRANSITION_TIME_MS;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sending msg: ONOFF SET %d\n", set_params.on_off);

    switch (button_number)
    {
        case 0:
        case 1:
            /* Demonstrate acknowledged transaction, using 1st client model instance */
            /* In this examples, users will not be blocked if the model is busy */
            (void)access_model_reliable_cancel(m_clients[0].model_handle);
            status = generic_onoff_client_set(&m_clients[0], &set_params, &transition_params);
            hal_led_pin_set(BSP_LED_0, set_params.on_off);
            break;

        case 2:
        case 3:
            /* Demonstrate un-acknowledged transaction, using 2nd client model instance */
            status = generic_onoff_client_set_unack(&m_clients[1], &set_params,
                                                    &transition_params, APP_UNACK_MSG_REPEAT_COUNT);
            hal_led_pin_set(BSP_LED_1, set_params.on_off);
            break;
    }

    switch (status)
    {
        case NRF_SUCCESS:
            break;

        case NRF_ERROR_NO_MEM:
        case NRF_ERROR_BUSY:
        case NRF_ERROR_INVALID_STATE:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Client %u cannot send\n", button_number);
            hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
            break;

        case NRF_ERROR_INVALID_PARAM:
            /* Publication not enabled for this client. One (or more) of the following is wrong:
             * - An application key is missing, or there is no application key bound to the model
             * - The client does not have its publication state set
             *
             * It is the provisioner that adds an application key, binds it to the model and sets
             * the model's publication state.
             */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Publication not configured for client %u\n", button_number);
            break;

        default:
            ERROR_CHECK(status);
            break;
    }
}

static void rtt_input_handler(int key)
{
    if (key >= '0' && key <= '3')
    {
        uint32_t button_number = key - '0';
        mesh_button_event_handler(button_number);
    }
}


static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");

    for (uint32_t i = 0; i < CLIENT_MODEL_INSTANCE_COUNT; ++i)
    {
        m_clients[i].settings.p_callbacks = &client_cbs;
        m_clients[i].settings.timeout = 0;
        m_clients[i].settings.force_segmented = APP_CONFIG_FORCE_SEGMENTATION;
        m_clients[i].settings.transmic_size = APP_CONFIG_MIC_SIZE;

        ERROR_CHECK(generic_onoff_client_init(&m_clients[i], i + 1));
    }
}

static void mesh_init(void)
{
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = NULL,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };
    ERROR_CHECK(mesh_stack_init(&init_params, &m_device_provisioned));
}

static void mesh_initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS | LOG_SRC_BEARER, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light Switch Client Demo -----\n");

    ERROR_CHECK(app_timer_init());
    hal_leds_init();

#if BUTTON_BOARD
    ERROR_CHECK(hal_buttons_init(mesh_button_event_handler));
#endif

    ble_stack_init();

#if MESH_FEATURE_GATT_ENABLED
    gap_params_init();
    conn_params_init();
#endif

    mesh_init();
}

static void mesh_start(void)
{
    rtt_input_enable(rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);

    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_complete_cb = provisioning_complete_cb,
            .prov_device_identification_start_cb = device_identification_start_cb,
            .prov_device_identification_stop_cb = NULL,
            .prov_abort_cb = provisioning_aborted_cb,
            .p_device_uri = EX_URI_LS_CLIENT
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }

    //mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());

    ERROR_CHECK(mesh_stack_start());

    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);
}

/**
 *------------------------------------------
 * End of mesh app 
 *------------------------------------------
 */
#endif

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
#if MQTT_TEST
    // Initialize.
    scheduler_init();
    sdk_log_init();
    leds_init();
    timers_init();
    iot_timer_init();
    button_init();

    static ipv6_medium_init_params_t ipv6_medium_init_params;
    memset(&ipv6_medium_init_params, 0x00, sizeof(ipv6_medium_init_params));
    ipv6_medium_init_params.ipv6_medium_evt_handler    = on_ipv6_medium_evt;
    ipv6_medium_init_params.ipv6_medium_error_handler  = on_ipv6_medium_error;

    err_code = ipv6_medium_init(&ipv6_medium_init_params,
                                IPV6_MEDIUM_ID_BLE,
                                &m_ipv6_medium);
    APP_ERROR_CHECK(err_code);

    eui48_t ipv6_medium_eui48;
    err_code = ipv6_medium_eui48_get(m_ipv6_medium.ipv6_medium_instance_id,
                                     &ipv6_medium_eui48);

    ipv6_medium_eui48.identifier[EUI_48_SIZE - 1] = 0x00;

    err_code = ipv6_medium_eui48_set(m_ipv6_medium.ipv6_medium_instance_id,
                                     &ipv6_medium_eui48);
    APP_ERROR_CHECK(err_code);

    ip_stack_init();

    NRF_LOG_INFO("Application started.");

    // Start execution.
    connectable_mode_enter();

    // Enter main loop.
    for (;;)
    {
        app_sched_execute();

        if (NRF_LOG_PROCESS() == false)
        {
            // Sleep waiting for an application event.
            err_code = sd_app_evt_wait();
            APP_ERROR_CHECK(err_code);
        }
    }
#endif
#if MESH_TEST
    mesh_initialize();
    mesh_start();

    // Enter main loop.
    for (;;)
    {
            sd_app_evt_wait();
    }
#endif
}
