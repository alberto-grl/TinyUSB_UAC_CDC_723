/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 * Copyright (c) 2020 Jerzy Kasenberg
 * Copyright (c) 2023 Alberto Garlassi
 *
 * A composite USB CDC + UAC device. Audio In (speaker) is available in audio_task.
 * Audio Out (microphone) generates a 432 Hz sinewave in tud_audio_tx_done_pre_load_cb.
 * For now fixed at 16 bit 48000. Set these parameters in Windows properties for the microphone peripheral.
 *
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */


/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tusb.h"

//#include "board.h"
#include "usb_descriptors.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// List of supported sample rates
#if defined(__RX__)
const uint32_t sample_rates[] = {44100, 48000};
#else
const uint32_t sample_rates[] = {44100, 48000, 88200, 96000};
#endif

uint32_t current_sample_rate  = 44100;

#define N_SAMPLE_RATES  TU_ARRAY_SIZE(sample_rates)

/* Blink pattern
 * - 25 ms   : streaming data
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum
{
	BLINK_STREAMING = 25,
	BLINK_NOT_MOUNTED = 250,
	BLINK_MOUNTED = 1000,
	BLINK_SUSPENDED = 2500,
};

enum
{
	VOLUME_CTRL_0_DB = 0,
	VOLUME_CTRL_10_DB = 2560,
	VOLUME_CTRL_20_DB = 5120,
	VOLUME_CTRL_30_DB = 7680,
	VOLUME_CTRL_40_DB = 10240,
	VOLUME_CTRL_50_DB = 12800,
	VOLUME_CTRL_60_DB = 15360,
	VOLUME_CTRL_70_DB = 17920,
	VOLUME_CTRL_80_DB = 20480,
	VOLUME_CTRL_90_DB = 23040,
	VOLUME_CTRL_100_DB = 25600,
	VOLUME_CTRL_SILENCE = 0x8000,
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/



/* USER CODE BEGIN PV */

PCD_HandleTypeDef hpcd_USB_OTG_HS;
static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;
// Audio controls
// Current states
int8_t mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];       // +1 for master channel 0
int16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];    // +1 for master channel 0

// Buffer for microphone data
int32_t mic_buf[CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ / 4];
// Buffer for speaker data
int32_t spk_buf[CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 4];
// Speaker data size received in the last frame
int spk_data_size;
// Resolution per format
const uint8_t resolutions_per_format[CFG_TUD_AUDIO_FUNC_1_N_FORMATS] = {CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX,
		CFG_TUD_AUDIO_FUNC_1_FORMAT_2_RESOLUTION_RX};
// Current resolution, update on format change
uint8_t current_resolution;

volatile uint32_t AudioCounter;
uint8_t BufferFillRequired = 1;
uint8_t SendGreetings = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USB_OTG_HS_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t board_millis(void);
//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
	blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
	blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
	(void)remote_wakeup_en;
	blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
	blink_interval_ms = BLINK_MOUNTED;
}

// Helper for clock get requests
static bool tud_audio_clock_get_request(uint8_t rhport, audio_control_request_t const *request)
{
	TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);

	if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ)
	{
		if (request->bRequest == AUDIO_CS_REQ_CUR)
		{
			TU_LOG1("Clock get current freq %lu\r\n", current_sample_rate);

			audio_control_cur_4_t curf = { (int32_t) tu_htole32(current_sample_rate) };
			return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &curf, sizeof(curf));
		}
		else if (request->bRequest == AUDIO_CS_REQ_RANGE)
		{
			audio_control_range_4_n_t(N_SAMPLE_RATES) rangef =
			{
					.wNumSubRanges = tu_htole16(N_SAMPLE_RATES)
			};
			TU_LOG1("Clock get %d freq ranges\r\n", N_SAMPLE_RATES);
			for(uint8_t i = 0; i < N_SAMPLE_RATES; i++)
			{
				rangef.subrange[i].bMin = (int32_t) sample_rates[i];
				rangef.subrange[i].bMax = (int32_t) sample_rates[i];
				rangef.subrange[i].bRes = 0;
				TU_LOG1("Range %d (%d, %d, %d)\r\n", i, (int)rangef.subrange[i].bMin, (int)rangef.subrange[i].bMax, (int)rangef.subrange[i].bRes);
			}

			return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &rangef, sizeof(rangef));
		}
	}
	else if (request->bControlSelector == AUDIO_CS_CTRL_CLK_VALID &&
			request->bRequest == AUDIO_CS_REQ_CUR)
	{
		audio_control_cur_1_t cur_valid = { .bCur = 1 };
		TU_LOG1("Clock get is valid %u\r\n", cur_valid.bCur);
		return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_valid, sizeof(cur_valid));
	}
	TU_LOG1("Clock get request not supported, entity = %u, selector = %u, request = %u\r\n",
			request->bEntityID, request->bControlSelector, request->bRequest);
	return false;
}

// Helper for clock set requests
static bool tud_audio_clock_set_request(uint8_t rhport, audio_control_request_t const *request, uint8_t const *buf)
{
	(void)rhport;

	TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);
	TU_VERIFY(request->bRequest == AUDIO_CS_REQ_CUR);

	if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ)
	{
		TU_VERIFY(request->wLength == sizeof(audio_control_cur_4_t));

		current_sample_rate = (uint32_t) ((audio_control_cur_4_t const *)buf)->bCur;

		TU_LOG1("Clock set current freq: %ld\r\n", current_sample_rate);

		return true;
	}
	else
	{
		TU_LOG1("Clock set request not supported, entity = %u, selector = %u, request = %u\r\n",
				request->bEntityID, request->bControlSelector, request->bRequest);
		return false;
	}
}

// Helper for feature unit get requests
static bool tud_audio_feature_unit_get_request(uint8_t rhport, audio_control_request_t const *request)
{
	TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);

	if (request->bControlSelector == AUDIO_FU_CTRL_MUTE && request->bRequest == AUDIO_CS_REQ_CUR)
	{
		audio_control_cur_1_t mute1 = { .bCur = mute[request->bChannelNumber] };
		TU_LOG1("Get channel %u mute %d\r\n", request->bChannelNumber, mute1.bCur);
		return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &mute1, sizeof(mute1));
	}
	else if (UAC2_ENTITY_SPK_FEATURE_UNIT && request->bControlSelector == AUDIO_FU_CTRL_VOLUME)
	{
		if (request->bRequest == AUDIO_CS_REQ_RANGE)
		{
			audio_control_range_2_n_t(1) range_vol = {
					.wNumSubRanges = tu_htole16(1),
					.subrange[0] = { .bMin = tu_htole16(-VOLUME_CTRL_50_DB), tu_htole16(VOLUME_CTRL_0_DB), tu_htole16(256) }
			};
			TU_LOG1("Get channel %u volume range (%d, %d, %u) dB\r\n", request->bChannelNumber,
					range_vol.subrange[0].bMin / 256, range_vol.subrange[0].bMax / 256, range_vol.subrange[0].bRes / 256);
			return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &range_vol, sizeof(range_vol));
		}
		else if (request->bRequest == AUDIO_CS_REQ_CUR)
		{
			audio_control_cur_2_t cur_vol = { .bCur = tu_htole16(volume[request->bChannelNumber]) };
			TU_LOG1("Get channel %u volume %d dB\r\n", request->bChannelNumber, cur_vol.bCur / 256);
			return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_vol, sizeof(cur_vol));
		}
	}
	TU_LOG1("Feature unit get request not supported, entity = %u, selector = %u, request = %u\r\n",
			request->bEntityID, request->bControlSelector, request->bRequest);

	return false;
}

// Helper for feature unit set requests
static bool tud_audio_feature_unit_set_request(uint8_t rhport, audio_control_request_t const *request, uint8_t const *buf)
{
	(void)rhport;

	TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);
	TU_VERIFY(request->bRequest == AUDIO_CS_REQ_CUR);

	if (request->bControlSelector == AUDIO_FU_CTRL_MUTE)
	{
		TU_VERIFY(request->wLength == sizeof(audio_control_cur_1_t));

		mute[request->bChannelNumber] = ((audio_control_cur_1_t const *)buf)->bCur;

		TU_LOG1("Set channel %d Mute: %d\r\n", request->bChannelNumber, mute[request->bChannelNumber]);

		return true;
	}
	else if (request->bControlSelector == AUDIO_FU_CTRL_VOLUME)
	{
		TU_VERIFY(request->wLength == sizeof(audio_control_cur_2_t));

		volume[request->bChannelNumber] = ((audio_control_cur_2_t const *)buf)->bCur;

		TU_LOG1("Set channel %d volume: %d dB\r\n", request->bChannelNumber, volume[request->bChannelNumber] / 256);

		return true;
	}
	else
	{
		TU_LOG1("Feature unit set request not supported, entity = %u, selector = %u, request = %u\r\n",
				request->bEntityID, request->bControlSelector, request->bRequest);
		return false;
	}
}

//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
	audio_control_request_t const *request = (audio_control_request_t const *)p_request;

	if (request->bEntityID == UAC2_ENTITY_CLOCK)
		return tud_audio_clock_get_request(rhport, request);
	if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT)
		return tud_audio_feature_unit_get_request(rhport, request);
	else
	{
		TU_LOG1("Get request not handled, entity = %d, selector = %d, request = %d\r\n",
				request->bEntityID, request->bControlSelector, request->bRequest);
	}
	return false;
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf)
{
	audio_control_request_t const *request = (audio_control_request_t const *)p_request;

	if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT)
		return tud_audio_feature_unit_set_request(rhport, request, buf);
	if (request->bEntityID == UAC2_ENTITY_CLOCK)
		return tud_audio_clock_set_request(rhport, request, buf);
	TU_LOG1("Set request not handled, entity = %d, selector = %d, request = %d\r\n",
			request->bEntityID, request->bControlSelector, request->bRequest);

	return false;
}

bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
	(void)rhport;

	uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
	uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

	if (ITF_NUM_AUDIO_STREAMING_SPK == itf && alt == 0)
		blink_interval_ms = BLINK_MOUNTED;

	return true;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
	(void)rhport;
	uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
	uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

	TU_LOG2("Set interface %d alt %d\r\n", itf, alt);
	if (ITF_NUM_AUDIO_STREAMING_SPK == itf && alt != 0)
		blink_interval_ms = BLINK_STREAMING;

	// Clear buffer when streaming format is changed
	spk_data_size = 0;
	if(alt != 0)
	{
		current_resolution = resolutions_per_format[alt-1];
	}

	return true;
}

bool tud_audio_rx_done_pre_read_cb(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting)
{
	(void)rhport;
	(void)func_id;
	(void)ep_out;
	(void)cur_alt_setting;

	spk_data_size = tud_audio_read(spk_buf, n_bytes_received);
	return true;
}

bool tud_audio_tx_done_pre_load_cb(uint8_t rhport, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting)
{
	(void)rhport;
	(void)itf;
	(void)ep_in;
	(void)cur_alt_setting;
	int16_t *dst = (int16_t*)mic_buf;

	for (uint16_t i = 0; i < 48000/1000; i++ )
			{
				*dst ++ = (int16_t)(20000.0 * sin(432.0 * 6.28 * AudioCounter++ / 48000));
			}
	tud_audio_write((uint8_t *)mic_buf, (uint16_t) (2 * 48000 /1000));



	// This callback could be used to fill microphone data separately
	return true;
}


//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
	(void) itf;
	(void) rts;

	// TODO set some indicator
	if ( dtr )
	{
		// Terminal connected
	}else
	{
		// Terminal disconnected
	}
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf)
{
	(void) itf;
}

void cdc_task(void)
{
	// connected() check for DTR bit
	// Most but not all terminal client set this when making connection
	 if ( tud_cdc_connected() )
	{
//TODO: terminal shows message only after a key is pressed. Why?
 		if (SendGreetings && tud_cdc_write_available())
		{
			tud_cdc_write("\n\rHello!\n\r", 8);
			tud_cdc_write_flush();
			SendGreetings = 0;
		}
		// connected and there are data available
		if ( tud_cdc_available() )
		{
			// read data
			char buf[64];
			uint32_t count = tud_cdc_read(buf, sizeof(buf));
			(void) count;

			// Echo back
			// Note: Skip echo by commenting out write() and write_flush()
			// for throughput test e.g
			//    $ dd if=/dev/zero of=/dev/ttyACM0 count=10000
			tud_cdc_write(buf, count);
			tud_cdc_write_flush();
		}
	}
}


//--------------------------------------------------------------------+
// AUDIO Task
//--------------------------------------------------------------------+

void audio_task(void)
{

#ifdef LOOPBACK_EXAMPLE

	// When new data arrived, copy data from speaker buffer, to microphone buffer
	// and send it over
	// Only support speaker & headphone both have the same resolution
	// If one is 16bit another is 24bit be care of LOUD noise !

#if 0
	//generate nasty noise
	mic_buf[0] = 20000;
	mic_buf[1] = 20000;
	mic_buf[2] = 20000;
	mic_buf[3] = 20000;
	tud_audio_write((uint8_t *)mic_buf, (uint16_t) (48));
#endif


	if (spk_data_size)
	{
		if (current_resolution == 16)
		{
			int16_t *src = (int16_t*)spk_buf;
			int16_t *limit = (int16_t*)spk_buf + spk_data_size / 2;
			int16_t *dst = (int16_t*)mic_buf;
			while (src < limit)
			{
				// Combine two channels into one
				int32_t left = *src++;
				int32_t right = *src++;
				*dst++ = (int16_t) ((left >> 1) + (right >> 1));
			}
			tud_audio_write((uint8_t *)mic_buf, (uint16_t) (spk_data_size / 2));
			spk_data_size = 0;
		}
		else if (current_resolution == 24)
		{
			int32_t *src = spk_buf;
			int32_t *limit = spk_buf + spk_data_size / 4;
			int32_t *dst = mic_buf;
			while (src < limit)
			{
				// Combine two channels into one
				int32_t left = *src++;
				int32_t right = *src++;
				*dst++ = (int32_t) ((uint32_t) ((left >> 1) + (right >> 1)) & 0xffffff00ul);
			}
			tud_audio_write((uint8_t *)mic_buf, (uint16_t) (spk_data_size / 2));
			spk_data_size = 0;
		}
	}
#endif
}


//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
	static uint32_t start_ms = 0;
	static bool led_state = false;

	// Blink every interval ms
	if (board_millis() - start_ms < blink_interval_ms) return;
	start_ms += blink_interval_ms;

	//  board_led_write(led_state);
	led_state = 1 - led_state;
}


/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_OTG_HS_PCD_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
   HAL_Delay(1000);
	//  board_init();
	TU_ASSERT(tusb_init());
	// init device stack on configured roothub port
//	tud_init(BOARD_TUD_RHPORT);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		tud_task();
		audio_task();
		cdc_task();
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 5;
	RCC_OscInitStruct.PLL.PLLN = 100;
	RCC_OscInitStruct.PLL.PLLP = 1;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
			|RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief USB_OTG_HS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_HS_PCD_Init(void)
{

	/* USER CODE BEGIN USB_OTG_HS_Init 0 */

	/* USER CODE END USB_OTG_HS_Init 0 */

	/* USER CODE BEGIN USB_OTG_HS_Init 1 */

	/* USER CODE END USB_OTG_HS_Init 1 */
	hpcd_USB_OTG_HS.Instance = USB_OTG_HS;
	hpcd_USB_OTG_HS.Init.dev_endpoints = 9;
	hpcd_USB_OTG_HS.Init.speed = PCD_SPEED_FULL;
	hpcd_USB_OTG_HS.Init.dma_enable = DISABLE;
	hpcd_USB_OTG_HS.Init.phy_itface = USB_OTG_EMBEDDED_PHY;
	hpcd_USB_OTG_HS.Init.Sof_enable = DISABLE;
	hpcd_USB_OTG_HS.Init.low_power_enable = DISABLE;
	hpcd_USB_OTG_HS.Init.lpm_enable = DISABLE;
	hpcd_USB_OTG_HS.Init.vbus_sensing_enable = DISABLE;
	hpcd_USB_OTG_HS.Init.use_dedicated_ep1 = DISABLE;
	hpcd_USB_OTG_HS.Init.use_external_vbus = DISABLE;
	if (HAL_PCD_Init(&hpcd_USB_OTG_HS) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USB_OTG_HS_Init 2 */

	/* USER CODE END USB_OTG_HS_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
