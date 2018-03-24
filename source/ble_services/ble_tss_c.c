/**
 * Author: Davide Giovanelli
 * April 2018
 *
 * File derived from Nordic examples (leds and button gatt client implementation)
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

#include "ble_tss_c.h"
#include "ble_srv_common.h"
#include "sdk_common.h"
#include "ble_types.h"
#include "ble_gattc.h"
#include "ble_hci.h"
#define NRF_LOG_MODULE_NAME "ble_tss_c     "
#include "nrf_log.h"

#define TX_BUFFER_MASK         0x03                  /**< TX Buffer mask, must be a mask of continuous zeroes, followed by continuous sequence of ones: 000...111. */
#define TX_BUFFER_SIZE         (TX_BUFFER_MASK + 1)  /**< Size of send buffer, which is 1 higher than the mask. */

#define WRITE_MESSAGE_LENGTH   NRF_BLE_GATT_MAX_MTU_SIZE//BLE_TSS_C_MAX_DATA_LEN//BLE_CCCD_VALUE_LEN    /**< Length of the write message for CCCD. */

#define BLE_UUID_TSS_CONFIG_CHAR      0x0501                      /**< The UUID of the config Characteristic. */
#define BLE_UUID_TSS_SPKR_CHAR        0x0502                      /**< The UUID of the speaker Characteristic. */
#define BLE_UUID_TSS_SPKR_STAT_CHAR   0x0503                      /**< The UUID of the speaker Status Characteristic. */
#define BLE_UUID_TSS_MIC_CHAR         0x0504                      /**< The UUID of the microphone Characteristic. */


#define BLE_TSS_MAX_RX_CHAR_LEN        BLE_TSS_C_MAX_DATA_LEN        /**< Maximum length of the RX Characteristic (in bytss). */
#define BLE_TSS_MAX_TX_CHAR_LEN        BLE_TSS_C_MAX_DATA_LEN        /**< Maximum length of the TX Characteristic (in bytss). */

// EF68xxxx-9B35-4933-9B10-52FFA9740042
#define TSS_BASE_UUID                  {0x42, 0x00, 0x74, 0xA9, 0xFF, 0x52, 0x10, 0x9B, 0x33, 0x49, 0x35, 0x9B, 0x00, 0x00, 0x68, 0xEF} /**< Used vendor specific UUID. */

typedef enum
{
    READ_REQ,      /**< Type identifying that this tx_message is a read request. */
    WRITE_REQ      /**< Type identifying that this tx_message is a write request. */
} tx_request_t;

/**@brief Structure for writing a message to the peer, i.e. CCCD.
 */
typedef struct
{
    uint8_t                  gattc_value[WRITE_MESSAGE_LENGTH];  /**< The message to write. */
    ble_gattc_write_params_t gattc_params;                       /**< The GATTC parameters for this message. */
} write_params_t;

/**@brief Structure for holding the data that will be transmitted to the connected central.
 */
typedef struct
{
    uint16_t     conn_handle;  /**< Connection handle to be used when transmitting this message. */
    tx_request_t type;         /**< Type of message. (read or write). */
    union
    {
        uint16_t       read_handle;  /**< Read request handle. */
        write_params_t write_req;    /**< Write request message. */
    } req;
} tx_message_t;

static tx_message_t  m_tx_buffer[TX_BUFFER_SIZE];  /**< Transmit buffer for the messages that will be transmitted to the central. */
static uint32_t      m_tx_insert_index = 0;        /**< Current index in the transmit buffer where the next message should be inserted. */
static uint32_t      m_tx_index        = 0;        /**< Current index in the transmit buffer containing the next message to be transmitted. */

ble_db_discovery_t m_db_disc;

/**@brief Function for passing any pending request from the buffer to the stack.
 */
static void tx_buffer_process(void)
{
    if (m_tx_index != m_tx_insert_index)
    {
        static uint32_t err_code;

        if (m_tx_buffer[m_tx_index].type == READ_REQ)
        {
            err_code = sd_ble_gattc_read(m_tx_buffer[m_tx_index].conn_handle,
                                         m_tx_buffer[m_tx_index].req.read_handle,
                                         0);
        }
        else
        {
            err_code = sd_ble_gattc_write(m_tx_buffer[m_tx_index].conn_handle,
                                          &m_tx_buffer[m_tx_index].req.write_req.gattc_params);
        }
        if (err_code == NRF_SUCCESS)
        {
//            NRF_LOG_DEBUG("SD Read/Write API returns Success..");
            m_tx_index++;
            m_tx_index &= TX_BUFFER_MASK;
        }
        else
        {
            NRF_LOG_DEBUG("SD Read/Write API returns error. This message sending will be "
                "attempted again..");
        }
    }
}

/**@brief Function for handling write response events.
 */
static void on_write_rsp(ble_tss_c_t * p_tss_c, ble_evt_t const * p_ble_evt)
{
    // Check if the event if on the link for this instance
    if (p_tss_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
    {
        return;
    }
    // Check if there is any message to be sent across to the peer and send it.
    tx_buffer_process();
}


/**@brief Function for handling Handle Value Notification received from the SoftDevice.
 */
static void on_hvx(ble_tss_c_t * p_tss_c, ble_evt_t const * p_ble_evt)
{
    // Check if the event is on the link for this instance
    if (p_tss_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
    {
        return;
    }

    if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_tss_c->peer_tss_db.speaker_status_handle)
    {
        if (p_ble_evt->evt.gattc_evt.params.hvx.len == sizeof(ble_tss_c_spkr_stat_t))
        {
            ble_tss_c_evt_t ble_tss_c_evt;

            ble_tss_c_evt.evt_type                   = BLE_TSS_C_EVT_SPEAKER_STATUS_NOTIFICATION;
            ble_tss_c_evt.conn_handle                = p_tss_c->conn_handle;
            ble_tss_c_evt.params.status = p_ble_evt->evt.gattc_evt.params.hvx.data[0];
            p_tss_c->evt_handler(p_tss_c, &ble_tss_c_evt);
        }
    }

    if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_tss_c->peer_tss_db.microphone_handle)
    {
        if (p_ble_evt->evt.gattc_evt.params.hvx.len <= BLE_TSS_C_MAX_DATA_LEN)
        {
            ble_tss_c_evt_t ble_tss_c_evt;

            ble_tss_c_evt.evt_type                   = BLE_TSS_C_EVT_MICROPHONE_NOTIFICATION;
            ble_tss_c_evt.conn_handle                = p_tss_c->conn_handle;
            for(uint16_t i = 0; i < p_ble_evt->evt.gattc_evt.params.hvx.len; i++){
            	ble_tss_c_evt.params.mic.frame[i] = p_ble_evt->evt.gattc_evt.params.hvx.data[i];
            }
            ble_tss_c_evt.params.mic.size = p_ble_evt->evt.gattc_evt.params.hvx.len;
            p_tss_c->evt_handler(p_tss_c, &ble_tss_c_evt);
        }
    }
}


/**@brief Function for handling Disconnected event received from the SoftDevice.
 *
 * @details This function check if the disconnect event is happening on the link
 *          associated with the current instance of the module, if so it will set its
 *          conn_handle to invalid.
 */
static void on_disconnected(ble_tss_c_t * p_tss_c, ble_evt_t const * p_ble_evt)
{
    if (p_tss_c->conn_handle == p_ble_evt->evt.gap_evt.conn_handle)
    {
        p_tss_c->conn_handle                    		= BLE_CONN_HANDLE_INVALID;
        p_tss_c->peer_tss_db.config_handle 				= BLE_GATT_HANDLE_INVALID;
        p_tss_c->peer_tss_db.speaker_data_handle      	= BLE_GATT_HANDLE_INVALID;
        p_tss_c->peer_tss_db.speaker_status_ccc_handle  = BLE_GATT_HANDLE_INVALID;
        p_tss_c->peer_tss_db.speaker_status_handle      = BLE_GATT_HANDLE_INVALID;
        p_tss_c->peer_tss_db.microphone_handle         	= BLE_GATT_HANDLE_INVALID;
    }
}

/**@brief Function for handling Connected event received from the SoftDevice.
 *
 * @details This function check if the connect event is happening on the link
 *          associated with the current instance of the module, if so it will set its
 *          conn_handle to invalid.
 */
static void on_connected(ble_tss_c_t * p_tss_c, ble_evt_t const * p_ble_evt)
{

    ret_code_t err_code;
	err_code = ble_tss_c_handles_assign(p_tss_c, p_ble_evt->evt.gap_evt.conn_handle, NULL);
	APP_ERROR_CHECK(err_code);
}

void ble_tss_on_db_disc_evt(ble_tss_c_t * p_tss_c, ble_db_discovery_evt_t const * p_evt)
{
    // Check if the Thingy sound Service was discovered.
    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE &&
        p_evt->params.discovered_db.srv_uuid.uuid == BLE_UUID_TSS_SERVICE &&
        p_evt->params.discovered_db.srv_uuid.type == p_tss_c->uuid_type)
    {
        ble_tss_c_evt_t evt;

        evt.evt_type    = BLE_TSS_C_EVT_DISCOVERY_COMPLETE;
        evt.conn_handle = p_evt->conn_handle;

        for (uint32_t i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
            const ble_gatt_db_char_t * p_char = &(p_evt->params.discovered_db.charateristics[i]);
            switch (p_char->characteristic.uuid.uuid)
            {
            	case BLE_UUID_TSS_CONFIG_CHAR:
            		p_tss_c->peer_tss_db.config_handle = p_char->characteristic.handle_value;
                	break;
            	case BLE_UUID_TSS_SPKR_CHAR:
            		p_tss_c->peer_tss_db.speaker_data_handle = p_char->characteristic.handle_value;
            		break;
                case BLE_UUID_TSS_SPKR_STAT_CHAR:
                	p_tss_c->peer_tss_db.speaker_status_handle = p_char->characteristic.handle_value;
                	p_tss_c->peer_tss_db.speaker_status_ccc_handle = p_char->cccd_handle;
                    break;
                case BLE_UUID_TSS_MIC_CHAR:
                	p_tss_c->peer_tss_db.microphone_handle     = p_char->characteristic.handle_value;
                	p_tss_c->peer_tss_db.microphone_ccc_handle = p_char->characteristic.handle_value;
                    break;

                default:
                    break;
            }
        }

        //If the instance has been assigned prior to db_discovery, assign the db_handles
        if (p_tss_c->conn_handle != BLE_CONN_HANDLE_INVALID)
        {
            if ((p_tss_c->peer_tss_db.config_handle         == BLE_GATT_HANDLE_INVALID)&&
                (p_tss_c->peer_tss_db.speaker_data_handle      == BLE_GATT_HANDLE_INVALID)&&
                (p_tss_c->peer_tss_db.speaker_status_ccc_handle == BLE_GATT_HANDLE_INVALID)&&
                (p_tss_c->peer_tss_db.speaker_status_handle == BLE_GATT_HANDLE_INVALID)&&
                (p_tss_c->peer_tss_db.microphone_handle == BLE_GATT_HANDLE_INVALID))
            {
                p_tss_c->peer_tss_db = evt.params.peer_db; //not clear what this does....
            }

            if ((p_tss_c->peer_tss_db.config_handle         != BLE_GATT_HANDLE_INVALID)&& //trigger the discovery complete event only if all characteristics are found
                (p_tss_c->peer_tss_db.speaker_data_handle      != BLE_GATT_HANDLE_INVALID)&&
                (p_tss_c->peer_tss_db.speaker_status_ccc_handle != BLE_GATT_HANDLE_INVALID)&&
                (p_tss_c->peer_tss_db.speaker_status_handle != BLE_GATT_HANDLE_INVALID)&&
                (p_tss_c->peer_tss_db.microphone_handle != BLE_GATT_HANDLE_INVALID))
            {
                p_tss_c->evt_handler(p_tss_c, &evt);
            }
        }

    }
}

uint32_t ble_tss_c_init(ble_tss_c_t * p_tss_c, ble_tss_c_init_t * p_ble_tss_c_init)
{
    ble_uuid_t    tss_uuid;

    VERIFY_PARAM_NOT_NULL(p_tss_c);
    VERIFY_PARAM_NOT_NULL(p_ble_tss_c_init);
    VERIFY_PARAM_NOT_NULL(p_ble_tss_c_init->evt_handler);

    p_tss_c->peer_tss_db.config_handle 				= BLE_GATT_HANDLE_INVALID;
    p_tss_c->peer_tss_db.speaker_data_handle      	= BLE_GATT_HANDLE_INVALID;
    p_tss_c->peer_tss_db.speaker_status_ccc_handle  = BLE_GATT_HANDLE_INVALID;
    p_tss_c->peer_tss_db.speaker_status_handle      = BLE_GATT_HANDLE_INVALID;
    p_tss_c->peer_tss_db.microphone_handle         	= BLE_GATT_HANDLE_INVALID;
    p_tss_c->conn_handle                    		= BLE_CONN_HANDLE_INVALID;
    p_tss_c->evt_handler                   			= p_ble_tss_c_init->evt_handler;

    tss_uuid.uuid = BLE_UUID_TSS_SERVICE;
    tss_uuid.type = p_tss_c->uuid_type;
    return ble_db_discovery_evt_register(&tss_uuid);
}

void ble_tss_c_on_ble_evt(ble_tss_c_t * p_tss_c, ble_evt_t * p_ble_evt) //(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_tss_c == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_HVX:
            on_hvx(p_tss_c, p_ble_evt);
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
            on_write_rsp(p_tss_c, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnected(p_tss_c, p_ble_evt);
            break;

        case BLE_GAP_EVT_CONNECTED:
        	on_connected(p_tss_c, p_ble_evt);
        	break;
        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
//            NRF_LOG_DEBUG("GATT Client Timeout.");
            sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
//            APP_ERROR_CHECK(err_code);
            break;
        case BLE_GATTC_EVT_EXCHANGE_MTU_RSP:
        	memset(&m_db_disc, 0x00, sizeof(ble_db_discovery_t));
        	err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
        	if (err_code != NRF_ERROR_BUSY) {
        		APP_ERROR_CHECK(err_code);
        	}
        	break;
        default:
            break;
    }

    ble_db_discovery_on_ble_evt(&m_db_disc, p_ble_evt); //this probably should be in m_sound.c, but froim there m_db_disc is not accessible...
}

/**@brief Function for configuring the CCCD.
 *
 * @param[in] conn_handle The connection handle on which to configure the CCCD.
 * @param[in] handle_cccd The handle of the CCCD to be configured.
 * @param[in] enable      Whether to enable or disable the CCCD.
 *
 * @return NRF_SUCCESS if the CCCD configure was successfully sent to the peer.
 */
static uint32_t cccd_configure(uint16_t conn_handle, uint16_t handle_cccd, bool enable)
{
//    NRF_LOG_DEBUG("Configuring CCCD. CCCD Handle = %d, Connection Handle = %d",
//        handle_cccd,conn_handle);

    tx_message_t * p_msg;
    uint16_t       cccd_val = enable ? BLE_GATT_HVX_NOTIFICATION : 0;

    p_msg              = &m_tx_buffer[m_tx_insert_index++];
    m_tx_insert_index &= TX_BUFFER_MASK;

    p_msg->req.write_req.gattc_params.handle   = handle_cccd;
    p_msg->req.write_req.gattc_params.len      = BLE_CCCD_VALUE_LEN;
    p_msg->req.write_req.gattc_params.p_value  = p_msg->req.write_req.gattc_value;
    p_msg->req.write_req.gattc_params.offset   = 0;
    p_msg->req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_REQ;
    p_msg->req.write_req.gattc_value[0]        = LSB_16(cccd_val);
    p_msg->req.write_req.gattc_value[1]        = MSB_16(cccd_val);
    p_msg->conn_handle                         = conn_handle;
    p_msg->type                                = WRITE_REQ;

    tx_buffer_process();
    return NRF_SUCCESS;
}

uint32_t ble_tss_c_speaker_status_notif_enable(ble_tss_c_t * p_tss_c)
{
    VERIFY_PARAM_NOT_NULL(p_tss_c);

    if (p_tss_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    return cccd_configure(p_tss_c->conn_handle,
                          p_tss_c->peer_tss_db.speaker_status_ccc_handle,
                          true);
}

/**@brief Function for creating a message for writing the speaker frame.
 */

uint32_t ble_tss_c_speaker_frame_send(ble_tss_c_t * p_tss_c, uint8_t * p_data, uint16_t size)
{
    VERIFY_PARAM_NOT_NULL(p_tss_c);
    VERIFY_PARAM_NOT_NULL(p_data);
//TODO: check size param
    if (p_tss_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

//    NRF_LOG_DEBUG("writing frame");

    tx_message_t * p_msg;

    p_msg              = &m_tx_buffer[m_tx_insert_index++];
    m_tx_insert_index &= TX_BUFFER_MASK;

    p_msg->req.write_req.gattc_params.handle   = p_tss_c->peer_tss_db.speaker_data_handle;
    p_msg->req.write_req.gattc_params.len      = size;
    p_msg->req.write_req.gattc_params.p_value  = p_msg->req.write_req.gattc_value;
    p_msg->req.write_req.gattc_params.offset   = 0;
    p_msg->req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_CMD; //TODO: CHECK WRITE WITHOUT RESPONSE
    for(uint16_t i = 0; i < size && i < WRITE_MESSAGE_LENGTH; i++){
    	p_msg->req.write_req.gattc_value[i]	       = (uint8_t)p_data[i]; //TODO: CHECK DATA COPY BOUNDARIES
    }
    p_msg->conn_handle                         = p_tss_c->conn_handle;
    p_msg->type                                = WRITE_REQ;

    tx_buffer_process();
    return NRF_SUCCESS;
}

/**@brief Function for creating a message for writing the speaker frame.
 */

uint32_t ble_tss_c_config_send(ble_tss_c_t * p_tss_c, uint8_t * p_data)
{
    VERIFY_PARAM_NOT_NULL(p_tss_c);
    VERIFY_PARAM_NOT_NULL(p_data);
//TODO: check size param
    if (p_tss_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

//    NRF_LOG_DEBUG("writing frame");

    tx_message_t * p_msg;

    p_msg              = &m_tx_buffer[m_tx_insert_index++];
    m_tx_insert_index &= TX_BUFFER_MASK;

    p_msg->req.write_req.gattc_params.handle   = p_tss_c->peer_tss_db.config_handle;
    p_msg->req.write_req.gattc_params.len      = 2;
    p_msg->req.write_req.gattc_params.p_value  = p_msg->req.write_req.gattc_value;
    p_msg->req.write_req.gattc_params.offset   = 0;
    p_msg->req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_REQ; //TODO: CHECK WRITE WITHOUT RESPONSE
    p_msg->req.write_req.gattc_value[0]	       = p_data[0]; //TODO: CHECK DATA COPY BOUNDARIES
    p_msg->req.write_req.gattc_value[1]	       = p_data[1]; //TODO: CHECK DATA COPY BOUNDARIES
    p_msg->conn_handle                         = p_tss_c->conn_handle;
    p_msg->type                                = WRITE_REQ;

    tx_buffer_process();
    return NRF_SUCCESS;
}

uint32_t ble_tss_c_handles_assign(ble_tss_c_t    * p_tss_c,
                                  uint16_t         conn_handle,
                                  const tss_db_t * p_peer_handles)
{
    VERIFY_PARAM_NOT_NULL(p_tss_c);

    p_tss_c->conn_handle = conn_handle;
    if (p_peer_handles != NULL)
    {
        p_tss_c->peer_tss_db = *p_peer_handles;
    }
    return NRF_SUCCESS;
}
