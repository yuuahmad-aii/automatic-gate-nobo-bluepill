/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdio.h>
#include "FEE.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* konfigurasi amalat memory untuk pengaturan */
#define PERIOD_MAX_SPEED_ADDRESS ((uint32_t)0x08007000)          //$1 default 20 (hz)
#define PERIOD_MIN_SPEED_ADDRESS ((uint32_t)0x08007010)          //$2 default 300 (hz)
#define TIME_INCREMENTPULSEWIDTH_ADDRESS ((uint32_t)0x08007020)  //$3 default 30 (ms)
#define TIME_DECCREMENTPULSEWIDTH_ADDRESS ((uint32_t)0x08007030) //$4 default 30 (ms)
#define VALUE_INCREMENTPULSEWIDTH_ADDRESS ((uint32_t)0x08007040) //$5 default 1 (hz)
#define VALUE_DECREMENTPULSEWIDTH_ADDRESS ((uint32_t)0x08007050) //$6 default 1 (hz)
#define NYALAKAN_VERBOSE_OUTPUT_ADDRESS ((uint32_t)0x08007060)   //$7 default 0 (verbose mati)
#define BALIK_ARAH_GERBANG_ADDRESS ((uint32_t)0x08007070)        //$8 default 0 (arah tidak terbalik)
#define BALIK_ENABLE_GERBANG_ADDRESS ((uint32_t)0x08007080)      //$9 default 0 (arah tidak terbalik)
#define DEBOUNCE_DELAY_EXTI_ADDRESS ((uint32_t)0x08007090)      //$10 default 0 (durasi debounce limit)
#define PULL_OFF_TIME_ADDRESS ((uint32_t)0x080070A0)      //$11 default 0 (durasi debounce limit)

/* Konfigurasi Parameter */
#define STEPS_PER_MM 100  // Langkah per mm
#define PULSE_WIDTH_US 10 // Lebar pulsa minimal agar bisa dibaca stepper driver dalam us
#define ACCELERATION 2000 // nilai akselerasi dalam mm/s^2
#define MAX_SPEED 1000    // max speed dalam mm/s
#define MIN_SPEED 50      // speed minimal jika dekselerasi mencapai spd minimal namun belum sampai limit_min_max

/* definisi interrupt handler */
//#define debounce_delay_exti 600 // Konfigurasi debounce delay external interrupt(dalam milidetik)
#define LIMIT_MIN_MAX_PIN 0
#define LIMIT_ACC_DCC_PIN 1
#define REMOTE_PIN 2
#define GERBANG_TERBUKA 0
#define GERBANG_TERTUTUP 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
/* verbose output parameter */
uint8_t nyalakan_verbose_output = 0;            // default mati
uint8_t baca_dan_tampilkan_flash = 0;            // baca dan tampilkan nilai flash memory

// rentang pulse dari lambat sampai cepat (max speed)
uint32_t period_max_speed = 20;            // dalam us
uint32_t period_min_speed = 300;            // dalam us
uint32_t time_incrementPulseWidth = 30;            // dalam ms (untuk menentukan akselerasi secara bodoh)
uint32_t time_decrementPulseWidth = 30;            // dalam ms (untuk menentukan dekselerasi secara bodoh)
uint32_t value_incrementPulseWidth = 1;            // per time_incrementPulseWidth berapa nilai currentPeriod yang berkurang
uint32_t value_decrementPulseWidth = 1;            // per time_incrementPulseWidth berapa nilai currentPeriod yang berkurang
uint32_t currentPeriod = 0;             // Periode awal dalam mikrodetik

uint32_t last_updateTimePeriod = 0;            // Waktu terakhir pembaruan periode (u/ hal_gettick)
uint8_t period_increasing_decreasing = 0;            // Flag untuk mengatur pengurangan nilai period step

// handle variabel dan flag untuk gerbang
uint8_t gerbang_berjalan = 0;            // Flag untuk menunjukkan gerbang berjalan
uint8_t arah_gerbang = 0;              // Flag untuk menunjukkan gerbang berjalan
uint32_t last_timeGerbangBerhenti = 0;            // handle delay gerbang mati setelah beberapa saat
uint32_t time_disableGerbang = 2000;            // handle delay gerbang mati setelah beberapa saat
uint8_t balik_arahGerbang = 0;            // variabel untuk membalik arah gerbang
uint8_t balik_enableGerbang = 0;
uint32_t debounce_delay_exti = 0;
uint32_t pull_off_time = 0;
uint32_t buffer_pull_off_time = 0;
uint8_t pull_off_state = 0;

// variabel untuk pin input gerbang (remote, limit_min_max, limit_acc_dcc)
volatile uint32_t last_buttonPressTime[3] = { 0, 0, 0 };            // interrupt delay
volatile uint8_t state_limit[3] = { 0, 0, 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void delay_us(uint32_t);
void baca_nilai_pada_flash(void);
// void move_motor(uint32_t, uint8_t);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
 CDC_Transmit_FS((uint8_t*) ptr, len);
 return len;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
 MX_USB_DEVICE_Init();
 MX_TIM2_Init();
 MX_RTC_Init();
 MX_SPI2_Init();
 MX_FATFS_Init();
 /* USER CODE BEGIN 2 */
 baca_nilai_pada_flash();
 /* USER CODE END 2 */

 /* Infinite loop */
 /* USER CODE BEGIN WHILE */
 while (1) {
  /*fungsi print ke serial monitor*/
  if (baca_dan_tampilkan_flash == 1) {
   baca_nilai_pada_flash();
   char buffer[512];
   int len =
    snprintf(buffer, sizeof(buffer),
     "$1 = %ld (period_max_speed)\n$2 = %ld (period_min_speed)\n$3 = %ld (time_incrementPulseWidth)\n$4 = %ld (time_decrementPulseWidth)\n$5 = %ld (value_incrementPulseWidth)\n$6 = %ld (value_decrementPulseWidth)\n$7 = %d (nyalakan verbose output)\n$8 = %d (1 bearti arah gerbang terbalik)\n$9 = %d (1 bearti enable gerbang terbalik)\n$10 = %ld (nilai debounce delay)\n$11 = %ld (pull off time)\n",
     period_max_speed, period_min_speed, time_incrementPulseWidth, time_decrementPulseWidth,
     value_incrementPulseWidth, value_decrementPulseWidth, nyalakan_verbose_output,
     balik_arahGerbang, balik_enableGerbang, debounce_delay_exti, pull_off_time);

   CDC_Transmit_FS((uint8_t*) buffer, len);
   //
   //   printf("$1 = %lu\n$2 = %lu\n$3 = %lu\n$4 = %lu\n$5 = %lu\n$6 = %lu\n", set_waktu_timer_oli,
   //    set_waktu_pompa_oli_on, tipe_emulasi, auto_counter_delay, verbose_delay, arah_auto_counter);
   baca_dan_tampilkan_flash = 0;
  }
  if (nyalakan_verbose_output == 1) {
   printf("nilai: %d %d %d %d %d \t\n", state_limit[LIMIT_MIN_MAX_PIN],
    state_limit[LIMIT_ACC_DCC_PIN], state_limit[REMOTE_PIN], gerbang_berjalan,
    period_increasing_decreasing);
  }

  /* handle arah gerbang. berikut adalah truth table-nya :3*/
  /* 0==0 true
   * 1==0 false
   * 0==1 false
   * 1==1 true
   */
  arah_gerbang == balik_arahGerbang ?
   HAL_GPIO_WritePin(DIR_STEPPER_GPIO_Port, DIR_STEPPER_Pin, GPIO_PIN_RESET) :
   HAL_GPIO_WritePin(DIR_STEPPER_GPIO_Port, DIR_STEPPER_Pin, GPIO_PIN_SET);

  /* handle disable stepper setelah beberapa saat berhenti */
  if (!gerbang_berjalan && HAL_GetTick() - last_timeGerbangBerhenti >= time_disableGerbang) {
   time_disableGerbang = HAL_GetTick();            // Catat waktu pembaruan
   balik_enableGerbang ?
    HAL_GPIO_WritePin(ENABLE_STEPPER_GPIO_Port, ENABLE_STEPPER_Pin, GPIO_PIN_SET) :
    HAL_GPIO_WritePin(ENABLE_STEPPER_GPIO_Port, ENABLE_STEPPER_Pin, GPIO_PIN_RESET);
  }

  /* lakukan sesuatu acc, cc, dan dcc berdasar state_limit[LIMIT_ACC_DCC_PIN]*/
  if (state_limit[LIMIT_ACC_DCC_PIN] == 0) {
   period_increasing_decreasing = 1;            // Mulai akselerasi
  } else if (state_limit[LIMIT_ACC_DCC_PIN] == 1) {
   period_increasing_decreasing = 0;            // Mulai konstan speed
  } else {
   period_increasing_decreasing = 2;            // Mulai dekselerasi
  }

  /*jika limit_min_max tertrigger ketika gerbang berjalan, matikan state_limit[REMOTE_PIN] sehingga mematikan pulse*/
  if (state_limit[LIMIT_MIN_MAX_PIN]) {
   state_limit[LIMIT_ACC_DCC_PIN] = 0;
   state_limit[LIMIT_MIN_MAX_PIN] = 0;
   state_limit[REMOTE_PIN] = 0;            // reset flag trigger remote gerbang
  }

  /* pin remote gerbang tertrigger */
  if (state_limit[REMOTE_PIN]) {
   if (!gerbang_berjalan) {                                           // jika gerbang tidak berjalan
    pull_off_state = 0;            //matikan pull_off_state
    gerbang_berjalan = 1;                                               // Set flag jalankan gerbang
    balik_enableGerbang ?
     HAL_GPIO_WritePin(ENABLE_STEPPER_GPIO_Port, ENABLE_STEPPER_Pin, GPIO_PIN_RESET) :
     HAL_GPIO_WritePin(ENABLE_STEPPER_GPIO_Port, ENABLE_STEPPER_Pin, GPIO_PIN_SET);            // aktifkan driver
    state_limit[LIMIT_ACC_DCC_PIN] = 0;                                              // mulai dari 0
    currentPeriod = period_min_speed;                                  // Reset periode ke min speed
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);                                           // Mulai PWM
   }
  } else { /*flag state_limit[LIMIT_ACC_DCC_PIN] mati, berhentikan gerbang*/
   if (gerbang_berjalan) {            // hentikan gerbang
//    state_limit[LIMIT_ACC_DCC_PIN] = 0;
//    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4); /* Matikan PWM*/
//    /*pull off gerbang*/
//    HAL_Delay(200); /*jeda sebentar sebelum pull-off*/
//    /*ini kebalikan dari yang atas tadi*/
//    arah_gerbang == balik_arahGerbang ?
//     HAL_GPIO_WritePin(DIR_STEPPER_GPIO_Port, DIR_STEPPER_Pin, GPIO_PIN_SET) :
//     HAL_GPIO_WritePin(DIR_STEPPER_GPIO_Port, DIR_STEPPER_Pin, GPIO_PIN_RESET);
//    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);                                           // Mulai PWM
//    __HAL_TIM_SET_AUTORELOAD(&htim2, period_min_speed);
//    HAL_Delay(pull_off_time);
//    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4); /* Matikan PWM*/
//
//    /*disable stepper motor*/
//    HAL_Delay(500);
//    balik_enableGerbang ?
//     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET) :
//     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);            // Pastikan output low
//      /*buat gerbang tidak berjalan lagi*/
//    gerbang_berjalan = 0; /*Reset flag tombol*/
//    period_increasing_decreasing = 0; /* Hentikan pengurangan periode*/
    state_limit[LIMIT_ACC_DCC_PIN] = 0;
    gerbang_berjalan = 0;                                 // Reset flag tombol
    period_increasing_decreasing = 0;                     // Hentikan pengurangan periode
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);              // Matikan PWM
    balik_enableGerbang ?
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET) :
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);            // Pastikan output low
   }
  }

  /* Perbarui periode sinyal untuk pull-off */
//  if (pull_off_state == 1 && HAL_GetTick() - last_updateTimePeriod >= time_incrementPulseWidth) {            // Perbarui setiap 20 ms
//   last_updateTimePeriod = HAL_GetTick();            // Catat waktu pembaruan                                    // lakukan akselerasi
//   if (buffer_pull_off_time > 0) {
//    buffer_pull_off_time -= buffer_pull_off_time - 1;            // Kurangi periode secara bertahap
//    __HAL_TIM_SET_AUTORELOAD(&htim2, period_min_speed);            // Update ARR
//                                                                   //    jangan update duty karena driver selalu menerima pulse 10us
//                                                                   //    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, currentPeriod / 2); // Update duty cycle
//   } else {
//    gerbang_berjalan = 0;                                 // Reset flag tombol
//    period_increasing_decreasing = 0;                     // Hentikan pengurangan periode
//    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);              // Matikan PWM
//    balik_enableGerbang ?
//     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET) :
//     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);            // Pastikan output low
//
//    buffer_pull_off_time = 0;
//    pull_off_state = 0;            // Periode mencapai 20 µs, hentikan pengurangan
//   }
//  }

  /* Perbarui periode sinyal untuk akselerasi */
  if (period_increasing_decreasing == 1
   && HAL_GetTick() - last_updateTimePeriod >= time_incrementPulseWidth) {            // Perbarui setiap 20 ms
   last_updateTimePeriod = HAL_GetTick();            // Catat waktu pembaruan
   if (currentPeriod > period_max_speed) {                                     // lakukan akselerasi
    currentPeriod -= value_incrementPulseWidth;            // Kurangi periode secara bertahap
    __HAL_TIM_SET_AUTORELOAD(&htim2, currentPeriod - 1);            // Update ARR
                                                                    //    jangan update duty karena driver selalu menerima pulse 10us
                                                                    //    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, currentPeriod / 2); // Update duty cycle
   } else {
    period_increasing_decreasing = 0;            // Periode mencapai 20 µs, hentikan pengurangan
   }
  }

  /* Perbarui periode sinyal untuk dekselerasi */
  if (period_increasing_decreasing == 2
   && HAL_GetTick() - last_updateTimePeriod >= time_incrementPulseWidth) {            // Perbarui setiap 20 ms
   last_updateTimePeriod = HAL_GetTick();            // Catat waktu pembaruan
   if (currentPeriod < period_min_speed) {                                     // lakukan akselerasi
    currentPeriod += value_decrementPulseWidth;            // Kurangi periode secara bertahap
    __HAL_TIM_SET_AUTORELOAD(&htim2, currentPeriod - 1);            // Update ARR
                                                                    //    jangan update duty karena driver selalu menerima pulse 10us
                                                                    //    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, currentPeriod / 2); // Update duty cycle
   } else {
    period_increasing_decreasing = 0;            // Periode mencapai 20 µs, hentikan pengurangan
   }
  }
 }
 /* USER CODE END WHILE */

 /* USER CODE BEGIN 3 */
 /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
 RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
 RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
 RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

 /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
 RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
 RCC_OscInitStruct.HSEState = RCC_HSE_ON;
 RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
 RCC_OscInitStruct.HSIState = RCC_HSI_ON;
 RCC_OscInitStruct.LSIState = RCC_LSI_ON;
 RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
 RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
 RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
 if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
  Error_Handler();
 }

 /** Initializes the CPU, AHB and APB buses clocks
  */
 RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
  | RCC_CLOCKTYPE_PCLK2;
 RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
 RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
 RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
 RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

 if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
  Error_Handler();
 }
 PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_USB;
 PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
 PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
 if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
  Error_Handler();
 }
}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void) {

 /* USER CODE BEGIN RTC_Init 0 */

 /* USER CODE END RTC_Init 0 */

 RTC_TimeTypeDef sTime = { 0 };
 RTC_DateTypeDef DateToUpdate = { 0 };

 /* USER CODE BEGIN RTC_Init 1 */

 /* USER CODE END RTC_Init 1 */

 /** Initialize RTC Only
  */
 hrtc.Instance = RTC;
 hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
 hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
 if (HAL_RTC_Init(&hrtc) != HAL_OK) {
  Error_Handler();
 }

 /* USER CODE BEGIN Check_RTC_BKUP */

 /* USER CODE END Check_RTC_BKUP */

 /** Initialize RTC and set the Time and Date
  */
 sTime.Hours = 0x16;
 sTime.Minutes = 0x30;
 sTime.Seconds = 0x0;

 if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) {
  Error_Handler();
 }
 DateToUpdate.WeekDay = RTC_WEEKDAY_SUNDAY;
 DateToUpdate.Month = RTC_MONTH_JANUARY;
 DateToUpdate.Date = 0x12;
 DateToUpdate.Year = 0x25;

 if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK) {
  Error_Handler();
 }
 /* USER CODE BEGIN RTC_Init 2 */

 /* USER CODE END RTC_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

 /* USER CODE BEGIN SPI2_Init 0 */

 /* USER CODE END SPI2_Init 0 */

 /* USER CODE BEGIN SPI2_Init 1 */

 /* USER CODE END SPI2_Init 1 */
 /* SPI2 parameter configuration*/
 hspi2.Instance = SPI2;
 hspi2.Init.Mode = SPI_MODE_MASTER;
 hspi2.Init.Direction = SPI_DIRECTION_2LINES;
 hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
 hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
 hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
 hspi2.Init.NSS = SPI_NSS_SOFT;
 hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
 hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
 hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
 hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
 hspi2.Init.CRCPolynomial = 10;
 if (HAL_SPI_Init(&hspi2) != HAL_OK) {
  Error_Handler();
 }
 /* USER CODE BEGIN SPI2_Init 2 */

 /* USER CODE END SPI2_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

 /* USER CODE BEGIN TIM2_Init 0 */

 /* USER CODE END TIM2_Init 0 */

 TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
 TIM_MasterConfigTypeDef sMasterConfig = { 0 };
 TIM_OC_InitTypeDef sConfigOC = { 0 };

 /* USER CODE BEGIN TIM2_Init 1 */

 /* USER CODE END TIM2_Init 1 */
 htim2.Instance = TIM2;
 htim2.Init.Prescaler = 72 - 1;
 htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
 htim2.Init.Period = 300;
 htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
 htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
 if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
  Error_Handler();
 }
 sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
 if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
  Error_Handler();
 }
 if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
  Error_Handler();
 }
 sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
 sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
 if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
  Error_Handler();
 }
 sConfigOC.OCMode = TIM_OCMODE_PWM1;
 sConfigOC.Pulse = 10;
 sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
 sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
 if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
  Error_Handler();
 }
 /* USER CODE BEGIN TIM2_Init 2 */

 /* USER CODE END TIM2_Init 2 */
 HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
 GPIO_InitTypeDef GPIO_InitStruct = { 0 };
 /* USER CODE BEGIN MX_GPIO_Init_1 */
 /* USER CODE END MX_GPIO_Init_1 */

 /* GPIO Ports Clock Enable */
 __HAL_RCC_GPIOC_CLK_ENABLE();
 __HAL_RCC_GPIOD_CLK_ENABLE();
 __HAL_RCC_GPIOA_CLK_ENABLE();
 __HAL_RCC_GPIOB_CLK_ENABLE();

 /*Configure GPIO pin Output Level */
 HAL_GPIO_WritePin(LED_NOTIFIKASI_GPIO_Port, LED_NOTIFIKASI_Pin, GPIO_PIN_RESET);

 /*Configure GPIO pin Output Level */
 HAL_GPIO_WritePin(GPIOB, ENABLE_STEPPER_Pin | DIR_STEPPER_Pin | SD_CS_Pin, GPIO_PIN_RESET);

 /*Configure GPIO pin : LED_NOTIFIKASI_Pin */
 GPIO_InitStruct.Pin = LED_NOTIFIKASI_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 HAL_GPIO_Init(LED_NOTIFIKASI_GPIO_Port, &GPIO_InitStruct);

 /*Configure GPIO pin : REMOTE_INPUT_Pin */
 GPIO_InitStruct.Pin = REMOTE_INPUT_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
 GPIO_InitStruct.Pull = GPIO_PULLUP;
 HAL_GPIO_Init(REMOTE_INPUT_GPIO_Port, &GPIO_InitStruct);

 /*Configure GPIO pin : LIMIT_ACC_DCC_Pin */
 GPIO_InitStruct.Pin = LIMIT_ACC_DCC_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
 GPIO_InitStruct.Pull = GPIO_PULLUP;
 HAL_GPIO_Init(LIMIT_ACC_DCC_GPIO_Port, &GPIO_InitStruct);

 /*Configure GPIO pin : LIMIT_MIN_MAX_Pin */
 GPIO_InitStruct.Pin = LIMIT_MIN_MAX_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
 GPIO_InitStruct.Pull = GPIO_PULLUP;
 HAL_GPIO_Init(LIMIT_MIN_MAX_GPIO_Port, &GPIO_InitStruct);

 /*Configure GPIO pins : ENABLE_STEPPER_Pin DIR_STEPPER_Pin SD_CS_Pin */
 GPIO_InitStruct.Pin = ENABLE_STEPPER_Pin | DIR_STEPPER_Pin | SD_CS_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

 /* EXTI interrupt init*/
 HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
 HAL_NVIC_EnableIRQ(EXTI0_IRQn);

 HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
 HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

 /* USER CODE BEGIN MX_GPIO_Init_2 */
 /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Function to Set RTC Time */
void Set_RTC_Time(uint8_t hour, uint8_t minute, uint8_t second) {
 RTC_TimeTypeDef sTime = { 0 };
 sTime.Hours = hour;
 sTime.Minutes = minute;
 sTime.Seconds = second;

 if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
  Error_Handler();
 }
}

/* Function to Get RTC Time */
//void Get_RTC_Time(void) {
//    RTC_TimeTypeDef sTime = {0};
//    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
//
//    /* Send Time via USB CDC */
//    snprintf((char *)USB_TxBuffer, sizeof(USB_TxBuffer), "Time: %02d:%02d:%02d\r\n",
//             sTime.Hours, sTime.Minutes, sTime.Seconds);
//    CDC_Transmit_FS(USB_TxBuffer, strlen((char *)USB_TxBuffer));
//}
// baca nilai pada flash memory dan masukkan ke variabel
void baca_nilai_pada_flash(void) {
 FEE_ReadData(PERIOD_MAX_SPEED_ADDRESS, &period_max_speed, sizeof(uint32_t));
 FEE_ReadData(PERIOD_MIN_SPEED_ADDRESS, &period_min_speed, sizeof(uint32_t));
 FEE_ReadData(TIME_INCREMENTPULSEWIDTH_ADDRESS, &time_incrementPulseWidth, sizeof(uint32_t));
 FEE_ReadData(TIME_DECCREMENTPULSEWIDTH_ADDRESS, &time_decrementPulseWidth, sizeof(uint32_t));
 FEE_ReadData(VALUE_INCREMENTPULSEWIDTH_ADDRESS, &value_incrementPulseWidth, sizeof(uint32_t));
 FEE_ReadData(VALUE_DECREMENTPULSEWIDTH_ADDRESS, &value_decrementPulseWidth, sizeof(uint32_t));
 FEE_ReadData(NYALAKAN_VERBOSE_OUTPUT_ADDRESS, &nyalakan_verbose_output, sizeof(uint8_t));
 FEE_ReadData(BALIK_ARAH_GERBANG_ADDRESS, &balik_arahGerbang, sizeof(uint8_t));
 FEE_ReadData(BALIK_ENABLE_GERBANG_ADDRESS, &balik_enableGerbang, sizeof(uint8_t));
 FEE_ReadData(DEBOUNCE_DELAY_EXTI_ADDRESS, &debounce_delay_exti, sizeof(uint32_t));
 FEE_ReadData(PULL_OFF_TIME_ADDRESS, &pull_off_time, sizeof(uint32_t));
}

// untuk menerima pesan dari pc
void USB_CDC_RxHandler(uint8_t *Buf, uint32_t Len) {
 /* Pastikan string berakhir dengan null-terminator */
 Buf[Len] = '\0';

 /* Periksa apakah perintah dimulai dengan '$' */
 if (Buf[0] == '$') {
  char *token = strtok((char*) Buf, "=");            // Pisahkan berdasarkan '='
  if (token != NULL && strcmp(token, "$1") == 0) {
   /* Ambil nilai setelah '=' */
   char *value = strtok(NULL, "=");
   if (value != NULL) {
    /* Ubah nilai variabel mode_emulasi */
    period_max_speed = atoi(value);
    FEE_WriteData(PERIOD_MAX_SPEED_ADDRESS, &period_max_speed, sizeof(uint32_t));
    printf("periode max speed %ld\n", period_max_speed);
   }
  } else if (token != NULL && strcmp(token, "$2") == 0) {
   char *value = strtok(NULL, "=");
   if (value != NULL) {
    period_min_speed = atoi(value);
    FEE_WriteData(PERIOD_MIN_SPEED_ADDRESS, &period_min_speed, sizeof(uint32_t));
    printf("periode min speed ke %ld\n", period_min_speed);
   }
  } else if (token != NULL && strcmp(token, "$3") == 0) {
   char *value = strtok(NULL, "=");
   if (value != NULL) {
    time_incrementPulseWidth = atoi(value);
    FEE_WriteData(TIME_INCREMENTPULSEWIDTH_ADDRESS, &time_incrementPulseWidth, sizeof(uint32_t));
    printf("waktu untuk increment periode ke %ld\n", time_incrementPulseWidth);
   }
  } else if (token != NULL && strcmp(token, "$4") == 0) {
   char *value = strtok(NULL, "=");
   if (value != NULL) {
    time_decrementPulseWidth = atoi(value);
    FEE_WriteData(TIME_DECCREMENTPULSEWIDTH_ADDRESS, &time_decrementPulseWidth, sizeof(uint32_t));
    printf("waktu untuk decrement periode %ld\n", time_decrementPulseWidth);
   }
  } else if (token != NULL && strcmp(token, "$5") == 0) {
   char *value = strtok(NULL, "=");
   if (value != NULL) {
    value_incrementPulseWidth = atoi(value);
    FEE_WriteData(VALUE_INCREMENTPULSEWIDTH_ADDRESS, &value_incrementPulseWidth, sizeof(uint32_t));
    printf("nilai increment periode setiap waktu %ld\n", value_incrementPulseWidth);
   }
  } else if (token != NULL && strcmp(token, "$6") == 0) {
   char *value = strtok(NULL, "=");
   if (value != NULL) {
    value_decrementPulseWidth = atoi(value);
    FEE_WriteData(VALUE_DECREMENTPULSEWIDTH_ADDRESS, &value_decrementPulseWidth, sizeof(uint32_t));
    printf("nilai decrement periode setiap waktu %ld\n", value_decrementPulseWidth);
   }
  } else if (token != NULL && strcmp(token, "$7") == 0) {
   char *value = strtok(NULL, "=");
   if (value != NULL) {
    nyalakan_verbose_output = atoi(value);
    FEE_WriteData(NYALAKAN_VERBOSE_OUTPUT_ADDRESS, &nyalakan_verbose_output, sizeof(uint32_t));
    printf("verbose output %d\n", nyalakan_verbose_output);
   }
  } else if (token != NULL && strcmp(token, "$8") == 0) {
   char *value = strtok(NULL, "=");
   if (value != NULL) {
    balik_arahGerbang = atoi(value);
    FEE_WriteData(BALIK_ARAH_GERBANG_ADDRESS, &balik_arahGerbang, sizeof(uint32_t));
    printf("arah gerbang %d\n", balik_arahGerbang);
   }
  } else if (token != NULL && strcmp(token, "$9") == 0) {
   char *value = strtok(NULL, "=");
   if (value != NULL) {
    balik_enableGerbang = atoi(value);
    FEE_WriteData(BALIK_ENABLE_GERBANG_ADDRESS, &balik_enableGerbang, sizeof(uint32_t));
    printf("enable gerbang %d\n", balik_enableGerbang);
   }
  } else if (token != NULL && strcmp(token, "$10") == 0) {
   char *value = strtok(NULL, "=");
   if (value != NULL) {
    debounce_delay_exti = atoi(value);
    FEE_WriteData(DEBOUNCE_DELAY_EXTI_ADDRESS, &debounce_delay_exti, sizeof(uint32_t));
    printf("debounce delay %ld\n", debounce_delay_exti);
   }
  } else if (token != NULL && strcmp(token, "$11") == 0) {
   char *value = strtok(NULL, "=");
   if (value != NULL) {
    pull_off_time = atoi(value);
    FEE_WriteData(PULL_OFF_TIME_ADDRESS, &pull_off_time, sizeof(uint32_t));
    printf("pull off time %ld\n", pull_off_time);
   }
  } else if (token != NULL && strcmp(token, "$$") == 0) {
   // perlihatkan apa yang ada pada flash memory (simpanan pengaturan)
   baca_dan_tampilkan_flash = 1;
  } else printf("Perintah tidak valid\n");
 }
}

// Fungsi Delay Mikrodetik
void delay_us(uint32_t us) {
 uint32_t start = TIM2->CNT;
 while ((TIM2->CNT - start) < us);
}

// program membaca nilai interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
 if (GPIO_Pin == LIMIT_MIN_MAX_Pin) {                // Periksa apakah interrupt berasal dari tombol
  uint32_t current_time = HAL_GetTick();            // Dapatkan waktu saat ini
  if ((current_time - last_buttonPressTime[LIMIT_MIN_MAX_PIN]) >= debounce_delay_exti) {
   last_buttonPressTime[LIMIT_MIN_MAX_PIN] = current_time;            // Perbarui waktu terakhir tombol ditekan
   // Tindakan saat tombol ditekan
   state_limit[LIMIT_MIN_MAX_PIN] = 1;
   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);            // Toggle LED di PC13
   printf("limit_min_max %d\r\n", state_limit[LIMIT_MIN_MAX_PIN]);
  }
 }

 if (GPIO_Pin == LIMIT_ACC_DCC_Pin) {                // Periksa apakah interrupt berasal dari tombol
  uint32_t current_time = HAL_GetTick();            // Dapatkan waktu saat ini
  if ((current_time - last_buttonPressTime[LIMIT_ACC_DCC_PIN]) >= debounce_delay_exti) {
   last_buttonPressTime[LIMIT_ACC_DCC_PIN] = current_time;            // Perbarui waktu terakhir tombol ditekan
   // Tindakan saat tombol ditekan
   state_limit[LIMIT_ACC_DCC_PIN]++;
   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);            // Toggle LED di PC13
   printf("limit_acc_dcc %d\r\n", state_limit[LIMIT_ACC_DCC_PIN]);
  }
 }

 if (GPIO_Pin == REMOTE_INPUT_Pin) {                 // Periksa apakah interrupt berasal dari tombol
  uint32_t current_time = HAL_GetTick();            // Dapatkan waktu saat ini
  if ((current_time - last_buttonPressTime[REMOTE_PIN]) >= debounce_delay_exti) {
   last_buttonPressTime[REMOTE_PIN] = current_time;            // Perbarui waktu terakhir tombol ditekan
   // Tindakan saat tombol berubah keadaan (rising and falling edge detect)
   if (HAL_GPIO_ReadPin(REMOTE_INPUT_GPIO_Port, REMOTE_INPUT_Pin) == GPIO_PIN_RESET) {
    arah_gerbang = GERBANG_TERBUKA;
   } else {
    arah_gerbang = GERBANG_TERTUTUP;
   }
   state_limit[REMOTE_PIN] = 1;
   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);            // Toggle LED di PC13
   printf("remote_pin %d\r\n", state_limit[REMOTE_PIN]);
  }
 }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
 /* USER CODE BEGIN Error_Handler_Debug */
 /* User can add his own implementation to report the HAL error return state */
 __disable_irq();
 while (1) {
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
