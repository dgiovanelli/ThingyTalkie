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
/**@file
 *
 * @defgroup ble_tss_c Thingy sound service Client
 * @{
 * @ingroup  ble_sdk_srv
 * @brief    Sound service client
 *
 *          @endcode
 */

#ifndef BLE_TSS_C_H__
#define BLE_TSS_C_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "app_util_platform.h"
#include "ble_db_discovery.h"
#include "drv_audio_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_UUID_TSS_SERVICE 0x0500                      /**< The UUID of the Thingy Sound Service. */
#define BLE_TSS_C_MAX_DATA_LEN CONFIG_AUDIO_FRAME_SIZE_BYTES//(BLE_GATT_ATT_MTU_DEFAULT - 3) /**< Maximum length of data (in bytss) that can be transmitted to the peer by the Thingy Sound service module. */

#define BLE_UUID_TSS_CONFIG_CHAR      0x0501                      /**< The UUID of the config Characteristic. */
#define BLE_UUID_TSS_SPKR_CHAR        0x0502                      /**< The UUID of the speaker Characteristic. */
#define BLE_UUID_TSS_SPKR_STAT_CHAR   0x0503                      /**< The UUID of the speaker Status Characteristic. */
#define BLE_UUID_TSS_MIC_CHAR         0x0504                      /**< The UUID of the microphone Characteristic. */


#define BLE_TSS_MAX_RX_CHAR_LEN        BLE_TSS_C_MAX_DATA_LEN        /**< Maximum length of the RX Characteristic (in bytss). */
#define BLE_TSS_MAX_TX_CHAR_LEN        BLE_TSS_C_MAX_DATA_LEN        /**< Maximum length of the TX Characteristic (in bytss). */

typedef uint8_t ble_tss_c_spkr_stat_t;

/**@brief TSS Client event type. */
typedef enum
{
    BLE_TSS_C_EVT_DISCOVERY_COMPLETE = 1,
	BLE_TSS_C_EVT_SPEAKER_STATUS_NOTIFICATION,
	BLE_TSS_C_EVT_MICROPHONE_NOTIFICATION
} ble_tss_c_evt_type_t;


typedef struct
{
    uint8_t frame[BLE_TSS_C_MAX_DATA_LEN];
    uint16_t size;
} ble_tss_c_microphone_t;

/**@brief Structure containing the handles related to the Sound Service found on the peer. */
typedef struct
{
    uint16_t config_handle;
    uint16_t speaker_data_handle;
    uint16_t speaker_status_ccc_handle;
    uint16_t speaker_status_handle;
    uint16_t microphone_ccc_handle;
    uint16_t microphone_handle;
} tss_db_t;

/**@brief Sound event structure. */
typedef struct
{
    ble_tss_c_evt_type_t evt_type;    /**< Type of the event. */
    uint16_t             conn_handle; /**< Connection handle on which the event occured.*/
    union
    {
    	ble_tss_c_spkr_stat_t 	status;
        tss_db_t     			peer_db;
        ble_tss_c_microphone_t	mic;
    } params;
} ble_tss_c_evt_t;

// Forward declaration of the ble_tss_c_t type.
typedef struct ble_tss_c_s ble_tss_c_t;

/**@brief   Event handler type.
 *
 * @details This is the type of the event handler that should be provided by the application
 *          of this module in order to receive events.
 */
typedef void (* ble_tss_c_evt_handler_t) (ble_tss_c_t * p_ble_tss_c, ble_tss_c_evt_t * p_evt);

struct ble_tss_c_s
{
    uint16_t                conn_handle;
    tss_db_t                peer_tss_db;
    ble_tss_c_evt_handler_t evt_handler;
    uint8_t                 uuid_type;
};

typedef struct
{
    ble_tss_c_evt_handler_t evt_handler;
} ble_tss_c_init_t;




uint32_t ble_tss_c_init(ble_tss_c_t * p_ble_tss_c, ble_tss_c_init_t * p_ble_tss_c_init);


void ble_tss_c_on_ble_evt(ble_tss_c_t * p_tss_c, ble_evt_t * p_ble_evt);


uint32_t ble_tss_c_speaker_status_notif_enable(ble_tss_c_t * p_tss_c);


void ble_tss_on_db_disc_evt(ble_tss_c_t * p_ble_tss_c, const ble_db_discovery_evt_t * p_evt);


uint32_t ble_tss_c_handles_assign(ble_tss_c_t *    p_ble_tss_c,
                                  uint16_t         conn_handle,
                                  const tss_db_t * p_peer_handles);


uint32_t ble_tss_c_speaker_frame_send(ble_tss_c_t * p_tss_c, uint8_t * p_data, uint16_t size);
uint32_t ble_tss_c_config_send(ble_tss_c_t * p_tss_c, uint8_t * p_data);

#ifdef __cplusplus
}
#endif

#endif // BLE_TSS_C_H__

/** @} */
