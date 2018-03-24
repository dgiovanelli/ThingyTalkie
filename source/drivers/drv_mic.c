/*
  Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

  2. Redistributions in binary form, except as embedded into a Nordic
     Semiconductor ASA integrated circuit in a product or a software update for
     such product, must reproduce the above copyright notice, this list of
     conditions and the following disclaimer in the documentation and/or other
     materials provided with the distribution.

  3. Neither the name of Nordic Semiconductor ASA nor the names of its
     contributors may be used to endorse or promote products derived from this
     software without specific prior written permission.

  4. This software, with or without modification, must only be used with a
     Nordic Semiconductor ASA integrated circuit.

  5. Any software provided in binary form under this license must not be reverse
     engineered, decompiled, modified and/or disassembled.

  THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
  OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
  OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "drv_audio.h"
#include "drv_audio_anr.h"
#include "drv_audio_dsp.h"
#include "drv_audio_coder.h"
#include "app_scheduler.h"
#include "app_error.h"
#include "drv_ext_gpio.h"
#include "drv_mic.h"
#include "app_util_platform.h"
#define  NRF_LOG_MODULE_NAME "drv_mic       "

#include "nrf_log.h"
#include "macros_common.h"
#include "arm_math.h"

#ifndef ENABLE_ANTIALIAS_FILTER
#define ENABLE_ANTIALIAS_FILTER 1
#endif

STATIC_ASSERT(CONFIG_PDM_BUFFER_SIZE_SAMPLES == (1 * CONFIG_AUDIO_FRAME_SIZE_SAMPLES));

#if ENABLE_ANTIALIAS_FILTER
#define BLOCK_SIZE            CONFIG_AUDIO_FRAME_SIZE_SAMPLES
#define NUM_TAPS              128
/* -------------------------------------------------------------------
 * Declare State buffer of size (numTaps + blockSize - 1)
 * ------------------------------------------------------------------- */
static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];
const float32_t firCoeffs32[NUM_TAPS] = { -0.00447747997286188950, -0.00012045510180908808, 0.00454310393726971820, 0.00377034597426701000, -0.00168608715234542460, -0.00526630650180259350,
		-0.00247838609171590310, 0.00348017035552228550, 0.00537245304959082720, 0.00069842781173663564, -0.00502960791397936890, -0.00477238639411509910, 0.00140168654201718880,
		0.00610470499256695040, 0.00345222529836235710, -0.00359491746914852510, -0.00650477871994264300, -0.00148301004570898050, 0.00561421829161098600, 0.00608375128652872880,
		-0.00097786526492543626, -0.00717744590423238790, -0.00477208363438213610, 0.00369394697204989330, 0.00801563214995659050, 0.00259264628567927890, -0.00636525647445097210,
		-0.00790177958650672110, 0.00033122632901913789, 0.00865054180551085710, 0.00667716273403659230, -0.00377630515598106160, -0.01019371429190665800, -0.00427225632139206430,
		0.00742824092658464060, 0.01065097041261761300, 0.00071986980463119830, -0.01089657508211180400, -0.00971405111188628110, 0.00384119886660795870, 0.01373189569992302100,
		0.00712351930599795180, -0.00917274068389621090, -0.01543750818382963600, -0.00266251058872246910, 0.01495365238074264400, 0.01546033548012208500, -0.00388800314202326910,
		-0.02080506356415224800, -0.01312468807235609400, 0.01288140432354697800, 0.02632182527795706400, 0.00740061380105553210, -0.02518725778192337400, -0.03110740977712250800,
		0.00391626723146844100, 0.04353879639374346600, 0.03480886725968031900, -0.02805354242998497200, -0.07986171725909514700, -0.03714842583792017000, 0.11583618674786206000,
		0.29401748366346087000, 0.37294329894961364000, 0.29401748366346087000, 0.11583618674786206000, -0.03714842583792017000, -0.07986171725909514700, -0.02805354242998497200,
		0.03480886725968031900, 0.04353879639374346600, 0.00391626723146844100, -0.03110740977712250800, -0.02518725778192337400, 0.00740061380105553210, 0.02632182527795706400,
		0.01288140432354697800, -0.01312468807235609400, -0.02080506356415224800, -0.00388800314202326910, 0.01546033548012208500, 0.01495365238074264400, -0.00266251058872246910,
		-0.01543750818382963600, -0.00917274068389621090, 0.00712351930599795180, 0.01373189569992302100, 0.00384119886660795870, -0.00971405111188628110, -0.01089657508211180400,
		0.00071986980463119830, 0.01065097041261761300, 0.00742824092658464060, -0.00427225632139206430, -0.01019371429190665800, -0.00377630515598106160, 0.00667716273403659230,
		0.00865054180551085710, 0.00033122632901913789, -0.00790177958650672110, -0.00636525647445097210, 0.00259264628567927890, 0.00801563214995659050, 0.00369394697204989330,
		-0.00477208363438213610, -0.00717744590423238790, -0.00097786526492543626, 0.00608375128652872880, 0.00561421829161098600, -0.00148301004570898050, -0.00650477871994264300,
		-0.00359491746914852510, 0.00345222529836235710, 0.00610470499256695040, 0.00140168654201718880, -0.00477238639411509910, -0.00502960791397936890, 0.00069842781173663564,
		0.00537245304959082720, 0.00348017035552228550, -0.00247838609171590310, -0.00526630650180259350, -0.00168608715234542460, 0.00377034597426701000, 0.00454310393726971820,
		-0.00012045510180908808, -0.00447747997286188950, -0.00335081070860515120 };
#endif

typedef struct
{
    int16_t  buf[CONFIG_PDM_BUFFER_SIZE_SAMPLES];
    uint16_t samples;
    bool     free;
}pdm_buf_t;

#define PDM_BUF_NUM 6

static bool                   m_audio_enabled;          ///< Audio enabled flag.
static drv_mic_data_handler_t m_data_handler;
static pdm_buf_t              m_pdm_buf[PDM_BUF_NUM];

static void mic_power_on(void)
{
    uint32_t err_code;
    
    nrf_gpio_cfg_input(MIC_DOUT, NRF_GPIO_PIN_NOPULL);

    #if defined(THINGY_HW_v0_7_0)
        err_code = drv_ext_gpio_pin_clear(SX_MIC_PWR_CTRL);
    #elif defined(THINGY_HW_v0_8_0)
        err_code = drv_ext_gpio_pin_clear(SX_MIC_PWR_CTRL);
    #elif defined(THINGY_HW_v0_9_0)
        err_code = drv_ext_gpio_pin_clear(SX_MIC_PWR_CTRL);
    #else
        err_code = drv_ext_gpio_pin_set(SX_MIC_PWR_CTRL);
    #endif
    APP_ERROR_CHECK(err_code);
}


static void mic_power_off(void)
{
    uint32_t err_code;

    #if defined(THINGY_HW_v0_7_0)
        err_code = drv_ext_gpio_pin_set(SX_MIC_PWR_CTRL);
    #elif defined(THINGY_HW_v0_8_0)
        err_code = drv_ext_gpio_pin_set(SX_MIC_PWR_CTRL);
    #elif defined(THINGY_HW_v0_9_0)
        err_code = drv_ext_gpio_pin_set(SX_MIC_PWR_CTRL);
    #else
        err_code = drv_ext_gpio_pin_clear(SX_MIC_PWR_CTRL);
    #endif
    
    nrf_gpio_cfg_input(MIC_DOUT, NRF_GPIO_PIN_PULLDOWN);
    
    APP_ERROR_CHECK(err_code);
}


static void m_audio_process(void * p_event_data, uint16_t event_size)
{
    int16_t         * p_buffer;
    ret_code_t        status;
    m_audio_frame_t   frame_buf;
    pdm_buf_t       * p_pdm_buf = (pdm_buf_t *)(*(uint32_t *)p_event_data);

    APP_ERROR_CHECK_BOOL(p_event_data != NULL);
    APP_ERROR_CHECK_BOOL(event_size > 0);
    p_buffer = p_pdm_buf->buf;

#if CONFIG_AUDIO_EQUALIZER_ENABLED
    drv_audio_dsp_equalizer((q15_t *)p_buffer, CONFIG_AUDIO_FRAME_SIZE_SAMPLES);
#endif /* CONFIG_AUDIO_EQUALIZER_ENABLED */

#if CONFIG_AUDIO_GAIN_CONTROL_ENABLED
    drv_audio_dsp_gain_control((q15_t *)p_buffer, CONFIG_AUDIO_FRAME_SIZE_SAMPLES);
#endif /* CONFIG_AUDIO_GAIN_CONTROL_ENABLED */

    uint8_t nested;

    app_util_critical_region_enter(&nested);
	drv_audio_coder_encode(p_buffer, &frame_buf);


/* Data contained in p_pdm_buf is 16bitPCM@16kHz, we want 8bitPCM@8kHz
 * Here the data is downsampled, and if enabled, a low pass fir filter is applied to remove aliasing effect,
 * anyway given the low quality of audio, removing alias doesn't help that much.*/

	float32_t outputF32[p_pdm_buf->samples];
	uint8_t pcm_8bit[p_pdm_buf->samples / 2];

#if ENABLE_ANTIALIAS_FILTER
	static arm_fir_instance_f32 S;
	static uint8_t initialized = false;
	float32_t inputF32[p_pdm_buf->samples];

	if (!initialized) {
		arm_fir_init_f32(&S, NUM_TAPS, (float32_t *) &firCoeffs32[0], &firStateF32[0], p_pdm_buf->samples);
		initialized = true;
	}

	for (uint32_t i = 0; i < p_pdm_buf->samples; i++) {
		inputF32[i] = (float32_t) (p_pdm_buf->buf[i]);
	}

	arm_fir_f32(&S, inputF32, outputF32, p_pdm_buf->samples);
#else
	for (uint32_t i = 0; i < p_pdm_buf->samples; i++) {
		outputF32[i] = (float32_t) (p_pdm_buf->buf[i]);
	}
#endif

	//actual downsample to 8bit pcm 8kHz
	for (uint32_t i = 0; i < p_pdm_buf->samples; i += 2) {
		uint32_t sample = (int32_t) ((outputF32[i] * 256.0) / 65536.0)+128;
		pcm_8bit[i / 2] = sample;
	}

	p_pdm_buf->free = true;
    app_util_critical_region_exit(nested);

    frame_buf.pcm_data = pcm_8bit;
    frame_buf.pcm_data_len = p_pdm_buf->samples/2;

    // Schedule audio transmission. It cannot be done from this context.
    status = m_data_handler(&frame_buf);
    if (status != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("Cannot schedule audio frame transmission!\r\n");
        /*
         * Do not clear CONFIG_IO_DBG_PCM. This will make debugging pulse wider
         * than expected and easier to spot on the logic analyzer.
         */
    }
}




static void m_audio_buffer_handler(int16_t *p_buffer, uint16_t samples)
{
    uint32_t     err_code;
    pdm_buf_t  * p_pdm_buf = NULL;
    uint32_t     pdm_buf_addr;

    for(uint32_t i = 0; i < PDM_BUF_NUM; i++)
    {
        if ( m_pdm_buf[i].free == true )
        {
            m_pdm_buf[i].free    = false;
            m_pdm_buf[i].samples = samples;

            for (uint32_t j = 0; j < samples; j++)
            {
                m_pdm_buf[i].buf[j] = p_buffer[j];
            }

            p_pdm_buf = &m_pdm_buf[i];
            pdm_buf_addr = (uint32_t)&m_pdm_buf[i];

            break;
        }
    }

    if (p_pdm_buf != NULL)
    {
        err_code = app_sched_event_put(&pdm_buf_addr, sizeof(pdm_buf_t *), m_audio_process);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        NRF_LOG_WARNING("m_audio_buffer_handler: BUFFER FULL!!\r\n");
    }
}


uint32_t drv_mic_start(void)
{
    ret_code_t status;

    NRF_LOG_DEBUG("m_audio: Enabled\r\n");

    if(m_audio_enabled == true)
    {
        return NRF_SUCCESS;
    }

    mic_power_on();

    status = drv_audio_enable();
    if (status == NRF_SUCCESS)
    {
        m_audio_enabled = true;
    }

    return status;
}


uint32_t drv_mic_stop(void)
{
    ret_code_t status;

    NRF_LOG_DEBUG("m_audio: Disabled\r\n");

    if(m_audio_enabled == false)
    {
        return NRF_SUCCESS;
    }

    status = drv_audio_disable();
    if (status == NRF_SUCCESS)
    {
        m_audio_enabled = false;
    }

    mic_power_off();

    return status;
}


uint32_t drv_mic_init(drv_mic_data_handler_t data_handler)
{
    uint32_t err_code;

    m_audio_enabled = false;
    m_data_handler  = data_handler;

    for(uint32_t i = 0; i < PDM_BUF_NUM; i++)
    {
        m_pdm_buf[i].free = true;
    }

    err_code = drv_ext_gpio_cfg_output(SX_MIC_PWR_CTRL);
    RETURN_IF_ERROR(err_code);

    mic_power_off();

    return drv_audio_init(m_audio_buffer_handler);
}
