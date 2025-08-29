  ******************************************************************************
  * author        : VoltVersa
  * institution   : Thomas More 2025
  ******************************************************************************


/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief         : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cg9a01.h"

#include "font8x16.h"
#include "max30102_for_stm32_hal.h"
#include <stdio.h>     // for snprintf()
#include <stdbool.h>   // for bool, true, false
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

/* USER CODE BEGIN PV */
/* ===================== UI colors ===================== */
#define BACKGROUND  0x0010   // dark navy background
/* ===================== TMP117 (I2C temperature sensor) ===================== */
#define TMP117_ADDR_BASE 0x48
#define TMP117_ADDR (TMP117_ADDR_BASE << 1) // HAL uses 8-bit address
#define TMP117_REG_TEMP 0x00
#define TMP117_REG_CONFIG 0x01
#define TMP117_REG_ID 0x0F
#define TMP117_DRDY_BIT (1U << 13) // CONFIG bit13 = DRDY
#define TMP117_LSB_C (0.0078125f) // 1 LSB = 0.0078125 °C


/* ===================== Battery measurement (10k:10k divider) ===================== */
#define ADC_MAX 4095.0f
#define VDDA 3.300f   /
#define R_TOP_OHM 10000.0f
#define R_BOTTOM_OHM 10000.0f
#define DIV_GAIN ((R_TOP_OHM + R_BOTTOM_OHM) / R_BOTTOM_OHM) // = 2.0 with 10k:10k
#define NUM_SAMPLES 16 // boxcar average


/* ===================== Alerts thresholds & hysteresis ===================== */
#define TEMP_HIGH_C 38.0f
#define TEMP_HIGH_CLEAR 37.5f
#define TEMP_LOW_C 35.0f
#define TEMP_LOW_CLEAR 35.5f
#define BPM_HIGH 120.0f
#define BPM_HIGH_CLEAR 115.0f
#define BPM_LOW 50.0f
#define BPM_LOW_CLEAR 55.0f
#define BATTERY_LOW_PCT 20
#define BATTERY_LOW_CLEAR 25


#define VIB_MS 120 // vibration duration (ms)
#define ALERT_SNOOZE_MS 120000UL // 2 min snooze after ACK


/* ===================== Types & globals ===================== */
typedef enum {
ALERT_NONE = 0,
ALERT_TEMP_HIGH,
ALERT_TEMP_LOW,
ALERT_BPM_HIGH,
ALERT_BPM_LOW,
ALERT_BATT_LOW
} alert_t;


static alert_t alert_state = ALERT_NONE;
static uint32_t vib_off_tick = 0; // stopwatch for motor auto-OFF
static uint32_t alert_snooze_until = 0; // suppress alerts until this tick


// BPM smoothing
#define IR_BUFFER_SIZE 800
uint32_t ir_buffer[IR_BUFFER_SIZE];
uint32_t ir_index = 0;
float g_bpm = 0.0f;
float bpm_buffer[20] = {0};
int bpm_index = 0;
int bpm_count = 0;
uint32_t last_beat_time = 0;
#define MAX30205_ADDR       (0x48 << 1)  // 0x90 write, 0x91 read
#define MAX30205_TEMP_REG   0x00
char received_char;


// RTC
static RTC_TimeTypeDef nowTime;
static RTC_DateTypeDef nowDate;


// MAX30102 instance


// Display power & key debounce
static bool display_on = true;
static uint32_t btn_last_tick = 0;
static uint8_t btn_last_level = 1; // with pull-up, idle=1

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void calculate_bpm(uint32_t *buffer, uint32_t length);
volatile uint8_t uart_flag = 0;
static void UI_DrawStatic(void);
static HAL_StatusTypeDef TMP117_Read16(uint8_t reg, uint16_t *out);
static HAL_StatusTypeDef TMP117_Write16(uint8_t reg, uint16_t val);
static void TMP117_StartContinuous_1Hz(void);
static bool TMP117_WaitDRDY(uint32_t timeout_ms);
static float TMP117_ReadTemp_Sync(uint32_t timeout_ms);
static uint16_t TMP117_ReadID(void);
static void Display_Set(bool on);
static bool OnOffButton_Toggled(void);
static void Vibrate_Start(uint32_t ms);
static void Vibrate_Stop(void);
static void Vibrate_Task(void);
static alert_t DecideAlert(float tempC, float bpm, int bat_pct);
static float ReadVBat_V(void);
static int LipoPercentFromVoltage(float v);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ===================== TMP117 helpers ===================== */
static HAL_StatusTypeDef TMP117_Read16(uint8_t reg, uint16_t *out)
{
    uint8_t d[2];
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c2, TMP117_ADDR, reg,
                                            I2C_MEMADD_SIZE_8BIT, d, 2, HAL_MAX_DELAY);
    if (st == HAL_OK) *out = (uint16_t)((d[0] << 8) | d[1]);
    return st;
}

static HAL_StatusTypeDef TMP117_Write16(uint8_t reg, uint16_t val)
{
    uint8_t d[2] = { (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    return HAL_I2C_Mem_Write(&hi2c2, TMP117_ADDR, reg,
                              I2C_MEMADD_SIZE_8BIT, d, 2, HAL_MAX_DELAY);
}

// MOD=00 continuous, CONV=100 (1 Hz), AVG=00
static void TMP117_StartContinuous_1Hz(void)
{
    uint16_t cfg = (4U << 7); // bits [9:7] = 100 (1 Hz)
    (void)TMP117_Write16(TMP117_REG_CONFIG, cfg);
}

static bool TMP117_WaitDRDY(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < timeout_ms) {
        uint16_t cfg = 0;
        if (TMP117_Read16(TMP117_REG_CONFIG, &cfg) != HAL_OK) return false;
        if (cfg & TMP117_DRDY_BIT) return true; // fresh sample ready
        HAL_Delay(2);
    }
    return false;
}

static float TMP117_ReadTemp_Sync(uint32_t timeout_ms)
{
    if (!TMP117_WaitDRDY(timeout_ms)) return -100.0f;
    uint16_t raw = 0;
    if (TMP117_Read16(TMP117_REG_TEMP, &raw) != HAL_OK) return -100.0f;
    return (int16_t)raw * TMP117_LSB_C;
}

static uint16_t TMP117_ReadID(void)
{
    uint16_t id = 0; (void)TMP117_Read16(TMP117_REG_ID, &id); return id;
}

/* ===================== MAX30102 BPM (simple peak detector) ===================== */
bool detectPulse(uint32_t irValue)
{
    static uint32_t prev_ir_value = 0;
    static uint8_t falling = 0;
    static uint32_t lastBeat = 0;

    bool pulseDetected = false;

    if (irValue > 50000 && irValue < 300000) // instead of 100000
    {
        if (irValue > prev_ir_value)
        {
            falling = 0;
        }
        else if (irValue < prev_ir_value)
        {
            if (!falling)
            {
                // Detected peak
                uint32_t now = HAL_GetTick();
                uint32_t beat_interval = now - lastBeat;
                lastBeat = now;

                if (beat_interval > 300 && beat_interval < 1500)
                {
                    float bpm = 60000.0f / beat_interval;
                    bpm_buffer[bpm_index++] = bpm;
                    if (bpm_index >= 10) bpm_index = 0;
                    if (bpm_count < 10) bpm_count++;

                    float bpm_sum = 0.0f;
                    for (int i = 0; i < bpm_count; ++i) bpm_sum += bpm_buffer[i];
                    g_bpm = bpm_sum / bpm_count;
                    pulseDetected = true;
                }
                falling = 1;
            }
        }
    }

    prev_ir_value = irValue;

    return pulseDetected;
}
max30102_t max30102;

/* ===================== UI helpers ===================== */
static void UI_DrawStatic(void)
{
    ClearScreen2(BACKGROUND);
    GC9A01_Draw_String(20,  60, "BPM:");
    GC9A01_Draw_String(20,  80, "TEMP:");
    GC9A01_Draw_String(20, 100, "TIME:");
    GC9A01_Draw_String(20, 120, "DATE:");
    GC9A01_Draw_String(20, 140, "Vbat:");
    GC9A01_Draw_String(20, 160, "Batt:");
}

static void UI_ShowAlert(const char* line1, const char* line2)
{
    ClearScreen2(BACKGROUND);
    GC9A01_Draw_String(20,  90, line1);
    if (line2 && line2[0]) GC9A01_Draw_String(20, 110, line2);
}

/* ===================== Display backlight & key ===================== */
static void Display_Set(bool on)
{
    display_on = on;
    HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if (!on) { HAL_GPIO_WritePin(motor_GPIO_Port, motor_Pin, GPIO_PIN_RESET); vib_off_tick = 0; }
    if (on) UI_DrawStatic(); else ClearScreen2(BACKGROUND);
}

// Debounced, active-low push button -> returns true on press edge
static bool OnOffButton_Toggled(void)
{
    uint8_t level = HAL_GPIO_ReadPin(ONOFF_GPIO_Port, ONOFF_Pin); // 1=idle, 0=pressed
    uint32_t now = HAL_GetTick();
    if (level != btn_last_level) {
        if ((now - btn_last_tick) >= 40) {
            bool falling_edge = (btn_last_level == 1 && level == 0);
            btn_last_level = level; btn_last_tick = now;
            if (falling_edge) return true;
        }
    }
    return false;
}

/* ===================== Vibration motor ===================== */
static void Vibrate_Start(uint32_t ms)
{
    HAL_GPIO_WritePin(motor_GPIO_Port, motor_Pin, GPIO_PIN_SET);
    vib_off_tick = HAL_GetTick() + ms;
}
static void Vibrate_Stop(void)
{
    HAL_GPIO_WritePin(motor_GPIO_Port, motor_Pin, GPIO_PIN_RESET);
    vib_off_tick = 0;
}
static void Vibrate_Task(void)
{
    if (vib_off_tick && HAL_GetTick() >= vib_off_tick) {
        HAL_GPIO_WritePin(motor_GPIO_Port, motor_Pin, GPIO_PIN_RESET);
        vib_off_tick = 0;
    }
}

/* ===================== Alerts engine ===================== */
static alert_t DecideAlert(float tempC, float bpm, int bat_pct)
{
    switch (alert_state) {
        case ALERT_NONE:
            if (bat_pct <= BATTERY_LOW_PCT)            return ALERT_BATT_LOW;
            if (tempC > -50.0f && tempC >= TEMP_HIGH_C) return ALERT_TEMP_HIGH;
            if (tempC > -50.0f && tempC <= TEMP_LOW_C)  return ALERT_TEMP_LOW;
            if (bpm >= BPM_HIGH)                        return ALERT_BPM_HIGH;
            if (bpm > 1.0f && bpm <= BPM_LOW)          return ALERT_BPM_LOW;
            return ALERT_NONE;
        case ALERT_BATT_LOW: return (bat_pct >= BATTERY_LOW_CLEAR) ? ALERT_NONE : ALERT_BATT_LOW;
        case ALERT_TEMP_HIGH:return (tempC <  TEMP_HIGH_CLEAR)     ? ALERT_NONE : ALERT_TEMP_HIGH;
        case ALERT_TEMP_LOW: return (tempC >  TEMP_LOW_CLEAR)      ? ALERT_NONE : ALERT_TEMP_LOW;
        case ALERT_BPM_HIGH: return (bpm   <= BPM_HIGH_CLEAR)      ? ALERT_NONE : ALERT_BPM_HIGH;
        case ALERT_BPM_LOW:  return (bpm   >= BPM_LOW_CLEAR)       ? ALERT_NONE : ALERT_BPM_LOW;
    }
    return ALERT_NONE;
}

/* ===================== Battery helpers ===================== */
typedef struct { float v; int pct; } vp_t;
static const vp_t lipo_lut[] = {
    {4.20f, 100}, {4.15f, 95}, {4.10f, 90}, {4.05f, 85},
    {4.00f, 80},  {3.96f, 75}, {3.92f, 70}, {3.88f, 65},
    {3.85f, 60},  {3.83f, 55}, {3.80f, 50}, {3.78f, 45},
    {3.75f, 40},  {3.73f, 35}, {3.71f, 30}, {3.69f, 25},
    {3.67f, 20},  {3.64f, 15}, {3.61f, 10}, {3.50f, 0}
};

static int LipoPercentFromVoltage(float v)
{
    if (v >= lipo_lut[0].v) return 100;
    if (v <= lipo_lut[sizeof(lipo_lut)/sizeof(lipo_lut[0]) - 1].v) return 0;
    for (size_t i = 0; i < sizeof(lipo_lut)/sizeof(lipo_lut[0]) - 1; ++i) {
        if (v <= lipo_lut[i].v && v >= lipo_lut[i+1].v) {
            float dv   = lipo_lut[i].v   - lipo_lut[i+1].v;
            float dpct = (float)lipo_lut[i].pct - (float)lipo_lut[i+1].pct;
            float t    = (dv > 0.0f) ? (lipo_lut[i].v - v) / dv : 0.0f;
            int pct    = (int)((float)lipo_lut[i].pct - t * dpct + 0.5f);
            if (pct < 0) pct = 0; else if (pct > 100) pct = 100; return pct;
        }
    }
    return 0;
}

static float ReadVBat_V(void)
{
    /* Dummy read for settling */
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    (void)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    uint32_t sum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 10);
        sum += HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
    }
    float raw   = (float)(sum / NUM_SAMPLES);
    float v_adc = (raw / ADC_MAX) * VDDA   ;// voltage at ADC pin
    float v_bat = v_adc * DIV_GAIN;         // back to pack voltage
    return v_bat;
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);

 // HAL_UART_Receive_IT(&huart1, &received_char, 1); // Enable UART RX interrupt

  /* ---- Display init & static UI ---- */
    GC9A01_Initial();
    UI_DrawStatic();
    Display_Set(true);   // ensure backlight state matches and UI is drawn

    /* ---- MAX30102 init: basic SpO2 mode with modest currents ---- */
	  max30102_init(&max30102, &hi2c1);
	  max30102_reset(&max30102);
	  max30102_clear_fifo(&max30102);
	  max30102_set_fifo_config(&max30102, max30102_smp_ave_8, 1, 7);

	  // Sensor settings
	  max30102_set_led_pulse_width(&max30102, max30102_pw_16_bit);
	  max30102_set_adc_resolution(&max30102, max30102_adc_2048);
	  max30102_set_sampling_rate(&max30102, max30102_sr_50);
	  max30102_set_led_current_1(&max30102, 6.2);
	  max30102_set_led_current_2(&max30102, 15);

	  // Enter SpO2 mode
	  max30102_set_mode(&max30102, max30102_spo2);
	  max30102_set_a_full(&max30102, 1);

	  // Initiate 1 temperature measurement
	  max30102_set_die_temp_en(&max30102, 1);
	  max30102_set_die_temp_rdy(&max30102, 1);

	  uint8_t en_reg[2] = {0};
	  max30102_read(&max30102, 0x00, en_reg, 1);
    /* ---- TMP117 init: continuous 1 Hz + quick identity check ---- */
    TMP117_StartContinuous_1Hz();
    uint16_t tmp117_id = TMP117_ReadID();
    if (tmp117_id != 0x0117) {
        // If you see this, address wiring may be 0x49..0x4B or sensor not present.
        GC9A01_Draw_String(20, 160, "TMP117?");
    }
    uint32_t lastUpdate = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      // Button: ACK alert or toggle display
      if (OnOffButton_Toggled()) {
          if (alert_state != ALERT_NONE) {
              Vibrate_Stop();
              alert_snooze_until = HAL_GetTick() + ALERT_SNOOZE_MS;
              alert_state = ALERT_NONE;
              UI_DrawStatic();
          } else {
              Display_Set(!display_on);
          }
      }

      // Motor auto-off
      Vibrate_Task();

      // Keep sampling even if screen is off (we just skip drawing)
      // Read whatever the sensor has and process ALL non-zero IR samples
      max30102_read_fifo(&max30102);

      for (int i = 0; i < 50; ++i) {
          uint32_t ir  = max30102._ir_samples[i];
          if (ir == 0) continue;          // skip empty slots
          detectPulse(ir);
      }


      float tempC = TMP117_ReadTemp_Sync(1200);      // wait up to one 1 Hz cycle
      float v_bat = ReadVBat_V();
      static float filtered_vbat = -1.0f;

      if (filtered_vbat < 0.0f) {
          filtered_vbat = v_bat;
      } else if (fabsf(v_bat - filtered_vbat) > 0.15f) {  // >150 mV jump? snap to it
          filtered_vbat = v_bat;
      } else {
          filtered_vbat = 0.8f*filtered_vbat + 0.2f*v_bat; // faster settling
      }


      int battery_percent = LipoPercentFromVoltage(filtered_vbat);

      // Alert engine with snooze
      bool    snoozed    = (alert_snooze_until != 0U) && (HAL_GetTick() < alert_snooze_until);
      alert_t decided    = DecideAlert(tempC, g_bpm, battery_percent);
      alert_t new_alert  = snoozed ? ALERT_NONE : decided;

      if (new_alert != alert_state) {
          alert_state = new_alert;
          if (alert_state != ALERT_NONE) {
              Vibrate_Start(VIB_MS);
              char line1[32], line2[32] = {0};
              switch (alert_state) {
                  case ALERT_BATT_LOW:
                      snprintf(line1, sizeof(line1), "LOW BATTERY!");
                      snprintf(line2, sizeof(line2), "%d%%  (<=%d%%)", battery_percent, BATTERY_LOW_PCT);
                      break;
                  case ALERT_TEMP_HIGH:
                      snprintf(line1, sizeof(line1), "TEMP TOO HIGH!");
                      snprintf(line2, sizeof(line2), "%.2f C  (>%.1f)", tempC, TEMP_HIGH_C);
                      break;
                  case ALERT_TEMP_LOW:
                      snprintf(line1, sizeof(line1), "TEMP TOO LOW!");
                      snprintf(line2, sizeof(line2), "%.2f C  (<%.1f)", tempC, TEMP_LOW_C);
                      break;
                  case ALERT_BPM_HIGH:
                      snprintf(line1, sizeof(line1), "HIGH BPM!");
                      snprintf(line2, sizeof(line2), "%.1f  (>%.0f)", g_bpm, BPM_HIGH);
                      break;
                  case ALERT_BPM_LOW:
                      snprintf(line1, sizeof(line1), "LOW BPM!");
                      snprintf(line2, sizeof(line2), "%.1f  (<%.0f)", g_bpm, BPM_LOW);
                      break;
                  default: break;
              }
              UI_ShowAlert(line1, line2);
          } else {
              UI_DrawStatic();
          }
      }
      if (alert_snooze_until && HAL_GetTick() >= alert_snooze_until) alert_snooze_until = 0;

      // UI refresh (skip if screen off or in alert)
      if (display_on && alert_state == ALERT_NONE) {
          uint32_t now = HAL_GetTick();
          if ((now - lastUpdate) >= 500) {
              lastUpdate = now;
              HAL_RTC_GetTime(&hrtc, &nowTime, RTC_FORMAT_BIN);
              HAL_RTC_GetDate(&hrtc, &nowDate, RTC_FORMAT_BIN);
              char bpmStr[24], tempStr[24], timeStr[24], dateStr[24], vbatStr[24], batperStr[24];
              snprintf(bpmStr,   sizeof(bpmStr),   "%5.1f  ", g_bpm);
              if (tempC > -50.0f) snprintf(tempStr, sizeof(tempStr), "%6.2f C  ", tempC);
              else                 snprintf(tempStr, sizeof(tempStr), "  --      ");
              snprintf(timeStr,  sizeof(timeStr),  "%02u:%02u:%02u",
                       nowTime.Hours, nowTime.Minutes, nowTime.Seconds);
              snprintf(dateStr,  sizeof(dateStr),  "%02u-%02u-20%02u",
                       nowDate.Date, nowDate.Month, nowDate.Year);
              snprintf(vbatStr,  sizeof(vbatStr),  "%6.3f V  ", filtered_vbat);
              snprintf(batperStr,sizeof(batperStr),"%3d %%   ", battery_percent);
              GC9A01_Draw_String(90,  60, bpmStr);
              GC9A01_Draw_String(90,  80, tempStr);
              GC9A01_Draw_String(90, 100, timeStr);
              GC9A01_Draw_String(90, 120, dateStr);
              GC9A01_Draw_String(90, 140, vbatStr);
              GC9A01_Draw_String(90, 160, batperStr);
          }
      }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x4;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_SUNDAY;
  DateToUpdate.Month = RTC_MONTH_JUNE;
  DateToUpdate.Date = 0x15;
  DateToUpdate.Year = 0x25;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_BLK_Pin|LCD_CS_Pin|LCD_DC_Pin|LCD_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|motor_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_BLK_Pin LED_Pin motor_Pin LCD_CS_Pin
                           LCD_DC_Pin LCD_RST_Pin */
  GPIO_InitStruct.Pin = LCD_BLK_Pin|LED_Pin|motor_Pin|LCD_CS_Pin
                          |LCD_DC_Pin|LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ONOFF_Pin */
  GPIO_InitStruct.Pin = ONOFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ONOFF_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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




