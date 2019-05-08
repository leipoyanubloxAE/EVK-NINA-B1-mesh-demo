/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
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
 */

#include <stdint.h>
#include <string.h>

/* HAL */
#include "boards.h"
#include "nrf_delay.h"
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
#include "proxy.h"

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

/* Models */
#include "generic_onoff_server.h"

/* Message */
#include "generic_onoff_messages.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Example specific includes */
#include "app_config.h"
#include "example_common.h"
#include "nrf_mesh_config_examples.h"
#include "light_switch_example_common.h"
#include "app_onoff.h"
#include "ble_softdevice_support.h"

#define ONOFF_SERVER_0_LED          (BSP_LED_0)
#define APP_ONOFF_ELEMENT_INDEX     (0)

#define MAX_MSG_PAYLOAD 64
uint8_t m_msg_payload[MAX_MSG_PAYLOAD];

static bool m_device_provisioned;

APP_TIMER_DEF(m_server_timer_id);                              /**< Server timer. */
#define SERVER_TIMER_INTERVAL           APP_TIMER_TICKS(15000)      /**< Server timer interval (ticks). */

/*************************************************************************************************/
static void app_onoff_server_set_cb(const app_onoff_server_t * p_server, bool onoff);
static void app_onoff_server_get_cb(const app_onoff_server_t * p_server, bool * p_present_onoff);

/* Generic OnOff server structure definition and initialization */
APP_ONOFF_SERVER_DEF(m_onoff_server_0,
                     APP_CONFIG_FORCE_SEGMENTATION,
                     APP_CONFIG_MIC_SIZE,
                     app_onoff_server_set_cb,
                     app_onoff_server_get_cb)

/* Callback for updating the hardware state */
static void app_onoff_server_set_cb(const app_onoff_server_t * p_server, bool onoff)
{
    /* Resolve the server instance here if required, this example uses only 1 instance. */

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Setting GPIO value: %d\n", onoff)

    hal_led_pin_set(ONOFF_SERVER_0_LED, onoff);
}

/* Callback for reading the hardware state */
static void app_onoff_server_get_cb(const app_onoff_server_t * p_server, bool * p_present_onoff)
{
    /* Resolve the server instance here if required, this example uses only 1 instance. */

    *p_present_onoff = hal_led_pin_get(ONOFF_SERVER_0_LED);
}

static void app_model_init(void)
{
    /* Instantiate onoff server on element index APP_ONOFF_ELEMENT_INDEX */
    ERROR_CHECK(app_onoff_init(&m_onoff_server_0, APP_ONOFF_ELEMENT_INDEX));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App OnOff Model Handle: %d\n", m_onoff_server_0.server.model_handle);
}

/*************************************************************************************************/

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);
    /* This function may return if there are ongoing flash operations. */
    mesh_stack_device_reset();
}


static void config_server_get_status(void)
{
    uint32_t status = NRF_SUCCESS;
    dsm_handle_t addressList[10];
    uint32_t count=10;
    nrf_mesh_address_t address;
    uint8_t max_pub_sub_size = 14;
    uint8_t pub[max_pub_sub_size]; /* maximum 3 group addresses */
    uint8_t sub[max_pub_sub_size]; /* maximum 3 group addresses */
    uint8_t pubIdx=0, subIdx=0;

    memset(m_msg_payload, 0, MAX_MSG_PAYLOAD * sizeof(uint8_t));
    memset(pub, 0, max_pub_sub_size * sizeof(uint8_t));
    memset(sub, 0, max_pub_sub_size * sizeof(uint8_t));

    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);

    bool ledStatus = hal_led_pin_get(ONOFF_SERVER_0_LED);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "LED status: %d \n", ledStatus);

    status = dsm_address_get_all(addressList, &count);
    if(status == NRF_SUCCESS)
    {
        for(int i=0; i<count; i++)
        {
            if(dsm_address_get(addressList[i], &address)!=NRF_ERROR_NOT_FOUND)
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "dmsHandle: %d %s Address: type: 0x%x, value, 0x%x, virtual_uuid: 0x%x\n", addressList[i], dsm_address_is_rx(&address)?"Subscription":"Publication", address.type, address.value, address.p_virtual_uuid);
                if(dsm_address_is_rx(&address))
                {
                    if(subIdx!=0)
                    {
                        sprintf(sub + subIdx, "-");
                        subIdx++;
                    }
                    sprintf(sub + subIdx, "%4x", address.value);
                    subIdx+=4;
                }
                else
                {
                    if(pubIdx!=0)
                    {
                        sprintf(pub + pubIdx, "-");
                        pubIdx++;
                    }
                    sprintf(pub + pubIdx, "%4x", address.value);
                    pubIdx+=4;
                }
            }
            else
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "dmsHandle: %d, Address not found\n", addressList[i]);
            }
        }
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Failed to get all address\n");
    }

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "sub(%d) %s. pub(%d) %s.\n", subIdx, sub, pubIdx, pub);

    sprintf(m_msg_payload, "{\"node\":%04d,\"stat\":%d,\"lvl\":255,\"sub\":\"", node_address.address_start, ledStatus);
    strncat(m_msg_payload, sub, subIdx);
    strcat(m_msg_payload, "\",\"pub\":\"");
    strncat(m_msg_payload, pub, pubIdx);
    strcat(m_msg_payload, "\"}");
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "m_msg_payload is %s.\n", m_msg_payload);
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "config_server_evt_cb: event: 0x%x\n", p_evt->type);

    if (p_evt->type == CONFIG_SERVER_EVT_MODEL_SUBSCRIPTION_ADD)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "EVT_MODEL_SUBSCRIPTION_ADD: model_handle: 0x%x address_handle: 0x%x\n", p_evt->params.model_subscription_add.model_handle, p_evt->params.model_subscription_add.address_handle);
        config_server_get_status();
    }
    if (p_evt->type == CONFIG_SERVER_EVT_MODEL_SUBSCRIPTION_DELETE)
    {       
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "EVT_MODEL_SUBSCRIPTION_DELETE: model_handle: 0x%x address_handle: 0x%x\n", p_evt->params.model_subscription_add.model_handle, p_evt->params.model_subscription_add.address_handle);
        config_server_get_status();
    }
    if (p_evt->type == CONFIG_SERVER_EVT_MODEL_PUBLICATION_SET)
    {
        dsm_handle_t dsmHandle;
        access_model_publish_address_get(p_evt->params.model_publication_set.model_handle, &dsmHandle);
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "EVT_MODEL_PUBLICATION_SET: model_handle: 0x%x, dsmHandle: %d\n", p_evt->params.model_publication_set.model_handle, dsmHandle);
        config_server_get_status();
    }

    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
}

static void button_event_handler(uint32_t button_number)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);
    switch (button_number)
    {
        /* Pressing SW1 on the Development Kit will result in LED state to toggle and trigger
        the STATUS message to inform client about the state change. This is a demonstration of
        state change publication due to local event. */
        case 0:
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "User action \n");
            hal_led_pin_set(ONOFF_SERVER_0_LED, !hal_led_pin_get(ONOFF_SERVER_0_LED));
            app_onoff_status_publish(&m_onoff_server_0);
            break;
        }

        /* Initiate node reset */
        case 3:
        {
            /* Clear all the states to reset the node. */
            if (mesh_stack_is_device_provisioned())
            {
#if MESH_FEATURE_GATT_PROXY_ENABLED
                (void) proxy_stop();
#endif
                mesh_stack_config_clear();
                node_reset();
            }
            else
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "The device is unprovisioned. Resetting has no effect.\n");
            }
            break;
        }

        default:
            break;
    }
}

static void app_rtt_input_handler(int key)
{
    if (key >= '0' && key <= '4')
    {
        uint32_t button_number = key - '0';
        button_event_handler(button_number);
    }
}

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
    m_device_provisioned = true;
}

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");
    app_model_init();
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

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS | LOG_SRC_BEARER, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light Switch Server Demo -----\n");

    ERROR_CHECK(app_timer_init());
    hal_leds_init();

#if BUTTON_BOARD
    ERROR_CHECK(hal_buttons_init(button_event_handler));
#endif

    ble_stack_init();

#if MESH_FEATURE_GATT_ENABLED
    gap_params_init();
    conn_params_init();
#endif

    mesh_init();
}

static void start(void)
{
    rtt_input_enable(app_rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);

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
            .p_device_uri = EX_URI_LS_SERVER
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }
    else
    {
        //nrf_gpio_cfg_input(BUTTON_1,GPIO_PIN_CNF_PULL_Pullup);
        if (nrf_gpio_pin_read(BUTTON_1)==0)
        {
            mesh_stack_config_clear();
            nrf_delay_ms(500);
            node_reset();
        }
    }

    mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());

    ERROR_CHECK(mesh_stack_start());

    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);
}

static void server_publish_status(void)
{
#if 0
    access_message_tx_t msg =
    {
        .opcode = ACCESS_OPCODE_SIG(GENERIC_ONOFF_OPCODE_STATUS),
        .p_buffer = (const uint8_t *) &m_msg_payload,
        .length = m_onoff_server_0.state.remaining_time_ms > 0 ? GENERIC_ONOFF_STATUS_MAXLEN : GENERIC_ONOFF_STATUS_MINLEN,
        .force_segmented = m_onoff_server_0.server.settings.force_segmented,
        .transmic_size = m_onoff_server_0.server.settings.transmic_size
    };
    //config_server_get_status();
    //access_model_publish(m_onoff_server_0.server.model_handle, &msg);
#endif
    app_onoff_status_publish(&m_onoff_server_0);
}

static void server_timeout_handler(void * p_context)
{
    uint32_t err_code;

    if(m_device_provisioned)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Server timeout handler\n");
        server_publish_status();
    }
}

static void server_timer_start(void)
{
    ret_code_t err_code;
    err_code = app_timer_create(&m_server_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                server_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_server_timer_id, SERVER_TIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

int main(void)
{
    initialize();
    start();
    server_timer_start();

    for (;;)
    {
        (void)sd_app_evt_wait();
    }
}
