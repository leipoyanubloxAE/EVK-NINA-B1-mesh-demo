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
#include "simple_hal.h"
#include "app_timer.h"
#include "nrf_delay.h"

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

APP_TIMER_DEF(m_client_timer_id);                              /**< Client timer. */
#define CLIENT_TIMER_INTERVAL           APP_TIMER_TICKS(10)      /**< Client timer interval (ticks). */

/* Uart */
#include "app_uart.h"
#include "nrf_uart.h"
#include "nrf_uarte.h"

#include "config_client.h"

#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

uint8_t m_uart_data[UART_RX_BUF_SIZE];
uint8_t m_uart_data_size = 0;
bool m_uart_data_ready = false;

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

void putstring(char* s)
{
    uint32_t err_code;
    
    uint8_t len = strlen((char *) s);
    for (uint8_t i = 0; i < len; i++)
    {
        err_code = app_uart_put(s[i]);
        if(err_code) __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "app_uart_put failed: err_code=0x%x\n", err_code);
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
    uint8_t msg[64];

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

    memset(msg, 0, 64 * sizeof(uint8_t));
    sprintf(msg, "{\"node\":\"%04d\",\"stat\":\"%d\"}\n", p_meta->src.value, p_in->present_on_off);
    putstring(msg);
}

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

    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);

    status = dsm_address_get_all(addressList, &count);
    if(status == NRF_SUCCESS)
    {
        for(int i=0; i<count; i++)
        {
            if(dsm_address_get(addressList[i], &address)!=NRF_ERROR_NOT_FOUND)
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "dmsHandle: %d %s Address: type: 0x%x, value, 0x%x, virtual_uuid: 0x%x\n", addressList[i], dsm_address_is_rx(&address)?"Subscription":"Publication", address.type, address.value, address.p_virtual_uuid);
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
        button_event_handler(button_number);
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

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS | LOG_SRC_BEARER, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light Switch Client Demo -----\n");

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

void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&m_uart_data[index]));
            index++;

            if ((m_uart_data[index - 1] == '\n') ||
                (m_uart_data[index - 1] == '\r') ||
                (index >= UART_RX_BUF_SIZE))
            {
                if (index > 1)
                {
                    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Received data from UART (%d): %s\n", index, m_uart_data);
                    //__LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "data dump", m_uart_data, index);
                    m_uart_data_size = index-1;
                    m_uart_data_ready = true;
                }
                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            //APP_ERROR_HANDLER(p_event->data.error_communication);
            //NRF_LOG_ERROR("APP_UART_COMMUNICATION_ERROR");
            break;

        case APP_UART_FIFO_ERROR:
            //APP_ERROR_HANDLER(p_event->data.error_code);
            //NRF_LOG_ERROR("APP_UART_FIFO_ERROR");
            break;

        default:
            break;
    }
}

void uart_init(void* event_handle)
{
    uint32_t err_code;
    bool status;

    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          false,                      /*Flow control, enable/disable.*/
          false,                     /*Even parity if TRUE, no parity if FALSE.*/
          NRF_UART_BAUDRATE_115200
      };
                                                                                                   
    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         event_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);
    APP_ERROR_CHECK(err_code);
}

static void app_config_client_event_cb(config_client_event_type_t event_type, const config_client_event_t * p_event, uint16_t length)
{
    /* USER_NOTE: Do additional processing of config client events here if required */
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Config client event 0x%x\n", p_event->opcode);

}

void config_onoffserver(uint16_t target_addr, bool status)
{
    generic_onoff_set_params_t set_params;
    model_transition_t transition_params;
    static uint8_t tid = 0;
    uint32_t result = NRF_SUCCESS;
    uint8_t element_index = 1;

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "config_onoffserver: target addr: 0x%x status: %d\n", target_addr, status);

    dsm_handle_t appkeyList[10];
    uint16_t count=10;
    dsm_handle_t appkey;

    result = access_model_applications_get(m_clients[element_index].model_handle, appkeyList, &count);
    if(result != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "access_model_applications_get failed: 0x%x\n", result);
    }

    /* Validate and add the publish address to the DSM: */
    dsm_handle_t publish_address_handle = DSM_HANDLE_INVALID;
    nrf_mesh_address_t publish_address_stored;
    /* Check if given publish address is different than the currently assigned address */
    if (access_model_publish_address_get(m_clients[element_index].model_handle, &publish_address_handle) != NRF_SUCCESS)
    {
        result = dsm_address_publish_add(target_addr, &publish_address_handle);
    }
    else
    {
        if (dsm_address_get(publish_address_handle, &publish_address_stored) == NRF_SUCCESS)
        {

            if ((publish_address_stored.type == NRF_MESH_ADDRESS_TYPE_VIRTUAL) ||
                (publish_address_stored.type != NRF_MESH_ADDRESS_TYPE_VIRTUAL  &&
                 publish_address_stored.value != target_addr))
            {
                /* This should never assert */
                NRF_MESH_ASSERT(dsm_address_publish_remove(publish_address_handle) == NRF_SUCCESS);
                result = dsm_address_publish_add(target_addr, &publish_address_handle);
            }
            else
            {
                /* Use the retrieved publish_address_handle */
            }
        }
        else
        {
            result = dsm_address_publish_add(target_addr, &publish_address_handle);
        }
    }

    result = access_model_publish_address_set(m_clients[element_index].model_handle, publish_address_handle);
    if(result != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "access_model_publish_address_set failed. Result: 0x%x\n", result);
    }

    /* Get the application key handle for the application key to publish on: */
    dsm_handle_t publish_appkey_handle = dsm_appkey_index_to_appkey_handle(0);
    if (publish_appkey_handle == DSM_HANDLE_INVALID)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "dsm_appkey_index_to_appkey_handle failed\n");
    }
    result = access_model_publish_application_set(m_clients[element_index].model_handle, publish_appkey_handle);
    if (result != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "access_model_publish_application_set failed: 0x%x\n", result);
    }

    //config_server_get_status();

    set_params.on_off = status;
    set_params.tid = tid++;
    transition_params.delay_ms = APP_CONFIG_ONOFF_DELAY_MS;
    transition_params.transition_time_ms = APP_CONFIG_ONOFF_TRANSITION_TIME_MS;
    //generic_onoff_client_set_unack(&m_clients[element_index], &set_params, &transition_params, APP_UNACK_MSG_REPEAT_COUNT);
    (void)access_model_reliable_cancel(m_clients[element_index].model_handle);
    generic_onoff_client_set(&m_clients[element_index], &set_params, &transition_params);

}

int hexStringToNum(uint8_t s[], uint8_t len) 
{
    int c, n;
 
    n = 0;
 
    for (c = 0; c<len; c++) {
        if(s[c]>='a' && s[c]<='f') n = n * 16 + s[c] - 'a' + 10;
        else if (s[c]>='A' && s[c]<='F') n = n * 16 + s[c] - 'A' + 10;
        else if (s[c]>='0' && s[c]<='9') n = n * 16 + s[c] - '0';
    }
 
    return n;
}

/* string should have format as {"node":0005,"stat":1}*/
static bool extract_node_status(uint8_t* data, uint8_t length, uint16_t* node, bool* status)
{
    char *node_start, *node_end, *status_start, *status_end;

    if(node==NULL || status==NULL)
    {
        return false;
    }

    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "data: %s length: %d\n", data, length);

    node_start = strstr(data, ":") + 1; 
    node_end = strstr(node_start, ",") - 1;

    if(node_start==NULL || node_end == NULL)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "node_start %s NULL, node_end %s NULL\n", node_start==NULL? "is": "is not", node_end==NULL? "is" : "is not");
        return false;
    }
    while((*node_start == ' ' || *node_start == '"') && (node_start < node_end))
    {
        node_start++;
    }
    while((*node_end == ' ' || *node_end == '"') && (node_start < node_end))
    {
        node_end--;
    }
    if(node_start==node_end)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "node_start equals node_end\n");
        return false;
    }


    status_start = strstr(node_end, ":") + 1;
    status_end = strstr(status_start, "}");
    if(status_start==NULL || status_end==NULL)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "status_start %s NULL, status_end %s NULL\n", status_start==NULL? "is": "is not", status_end==NULL? "is" : "is not");
        return false;
    }
    while((*status_start == ' ' || *status_start == '"') && (status_start < status_end))
    {
        status_start++;
    }
    if(status_start==status_end)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "status_start equals status_end\n");
        return false;
    }

    *node = hexStringToNum(node_start, node_end-node_start+1);
    *status = hexStringToNum(status_start, 1) == 0 ? false : true;

    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "node_start: %s node_end: %s status_start: %s status_end: %s node: 0x%x status: %d\n", node_start, node_end, status_start, status_end, *node, *status);

    return true;
}

static void client_timeout_handler(void * p_context)
{
    uint32_t err_code;

    if(m_uart_data_ready)
    {
        m_uart_data_ready = false;
        uint16_t node;
        bool status;

        if(extract_node_status(m_uart_data, m_uart_data_size, &node, &status))
        {
            config_onoffserver(node, status);
        }
    }
}

static void client_timer_start(void)
{
    ret_code_t err_code;
    err_code = app_timer_create(&m_client_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                client_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_client_timer_id, CLIENT_TIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

int main(void)
{
    initialize();
    start();

    uart_init(uart_event_handle);
    client_timer_start();

    for (;;)
    {
        (void)sd_app_evt_wait();
    }
}
