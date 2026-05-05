/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "secure_telemetry.h"
#include "secure_link_config.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOLD_TEMP_THRESHOLD_C 24.0f
#define MOLD_HUMIDITY_THRESHOLD_PCT 80.0f
#define MOLD_TEMP_DELTA_C 2.0f
#define MOLD_HUMIDITY_DELTA_PCT 5.0f
#define FROST_TRIGGER_DELTA_C 2.0f
#define FLOW_TRIGGER_DELTA_LPM 2.0f
#define MOLD_PREDICTION_LOOKAHEAD_MS 180000U
#define FROST_PREDICTION_LOOKAHEAD_MS 180000U
#define FLOW_PREDICTION_LOOKAHEAD_MS 30000U
#define TEMP_TREND_NOISE_FLOOR_C 0.2f
#define HUMIDITY_TREND_NOISE_FLOOR_PCT 0.5f
#define FLOW_TREND_NOISE_FLOOR_LPM 0.2f
#define CLIMATE_EVENT_WINDOW_MS 60000U
#define DHT22_MIN_TEMPERATURE_C -40.0f
#define DHT22_MAX_TEMPERATURE_C 80.0f
#define DHT22_MAX_HUMIDITY_PCT 100.0f
#define CLIMATE_TREND_CONFIRM_SAMPLES 2U
#define BUZZER_ACTIVE_LEVEL GPIO_PIN_SET
#define BUZZER_BEEP_DURATION_MS 2500U
#define FLOW_RELAY_ACTIVE_LEVEL GPIO_PIN_RESET
#define FROST_RELAY_ACTIVE_LEVEL GPIO_PIN_RESET
#define STEPPER_STEP_INTERVAL_MS 4U
#define DHT22_SAMPLE_INTERVAL_MS 2000U
#define FLOW_SAMPLE_INTERVAL_MS 1000U
#define TELEMETRY_SEND_INTERVAL_MS 1000U
#define ESP_RX_BUFFER_SIZE 384U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

volatile float g_temperature_c = 0.0f;
volatile float g_humidity_pct = 0.0f;
volatile float g_flow_l_min = 0.0f;
volatile float g_flow_total_l = 0.0f;
volatile float g_flow_frequency_hz = 0.0f;
volatile uint32_t g_flow_pulse_count = 0U;
volatile uint8_t g_dht_valid = 0U;

static uint32_t flow_last_sample_tick = 0U;
static uint32_t flow_last_sample_pulses = 0U;
static uint32_t dht_last_sample_tick = 0U;
static float climate_reference_temperature_c = 0.0f;
static float climate_reference_humidity_pct = 0.0f;
static uint32_t climate_reference_tick = 0U;
static uint8_t climate_reference_valid = 0U;
static float frost_reference_temperature_c = 0.0f;
static uint32_t frost_reference_tick = 0U;
static uint8_t frost_reference_valid = 0U;
static float flow_reference_l_min = 0.0f;
static uint8_t flow_reference_valid = 0U;
static uint32_t telemetry_last_send_tick = 0U;
static uint8_t esp_tcp_connected = 0U;
static char esp_rx_line[ESP_RX_BUFFER_SIZE] = {0};
static uint8_t temperature_stepper_active = 0U;
static uint8_t flow_relay_active = 0U;
static uint8_t frost_relay_active = 0U;
static uint8_t flow_risk_predicted_state = 0U;
static uint8_t stepper_sequence_index = 0U;
static uint32_t stepper_last_step_tick = 0U;
static uint8_t buzzer_active = 0U;
static uint32_t buzzer_off_tick = 0U;
static uint8_t buzzer_mold_alarm_latched = 0U;
static uint8_t buzzer_frost_alarm_latched = 0U;
static uint8_t buzzer_flow_alarm_latched = 0U;
static uint8_t mold_alarm_active = 0U;
static uint8_t frost_alarm_active = 0U;
static uint8_t flow_alarm_active_state = 0U;
static float last_temperature_sample_c = 0.0f;
static float last_humidity_sample_pct = 0.0f;
static uint32_t last_dht_trend_tick = 0U;
static uint8_t dht_trend_valid = 0U;
static uint8_t mold_trend_rise_streak = 0U;
static uint8_t frost_trend_drop_streak = 0U;
static float last_flow_sample_l_min = 0.0f;
static uint32_t last_flow_trend_tick = 0U;
static uint8_t flow_trend_valid = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void Sensors_Init(void);
static void Sensors_Service(void);
static void ServiceDelay(uint32_t delay_ms);
static void DWT_Delay_Init(void);
static void DelayUs(uint32_t delay_us);
static void DHT22_SetPinOutput(void);
static void DHT22_SetPinInput(void);
static uint8_t DHT22_WaitForState(GPIO_PinState state, uint32_t timeout_us);
static uint8_t DHT22_Read(float *temperature_c, float *humidity_pct);
static void DHT22_Service(void);
static uint8_t DHT22_IsReadingPlausible(float temperature_c, float humidity_pct);
static void FlowSensor_Service(void);
static void Stepper_Init(void);
static void Stepper_Service(void);
static void Stepper_Start(void);
static void Stepper_Stop(void);
static void Buzzer_Init(void);
static void Buzzer_Service(void);
static void Buzzer_Set(uint8_t enabled);
static void Buzzer_Trigger(uint32_t duration_ms);
static void Relay_Init(void);
static void Relay_Service(void);
static void Relay_Set(uint8_t enabled);
static void FrostRelay_Set(uint8_t enabled);
static void Telemetry_Service(void);
static void ESP_FlushRx(void);
static uint8_t ESP_WaitForResponse(const char *expected_ok, const char *expected_alt_ok, const char *expected_fail, uint32_t timeout_ms);
static uint8_t ESP_SendCommandAndWait(const char *command, const char *expected_ok, const char *expected_alt_ok, const char *expected_fail, uint32_t timeout_ms);
static uint8_t ESP_InitModule(void);
static uint8_t ESP_EnsureTcpConnected(void);
static uint8_t ESP_SendPayload(const char *payload);
static void ESP_RecoverLink(void);
static void BuildSensorPayload(char *buffer, size_t buffer_len);
static uint8_t PredictMoldRisk(
    float temperature_c,
    float humidity_pct,
    uint32_t now,
    uint32_t elapsed_ms,
    float temp_delta_c,
    float humidity_delta_pct);
static uint8_t PredictFrostRisk(
    float temperature_c,
    uint32_t now,
    uint32_t elapsed_ms,
    float temp_delta_c);
static uint8_t PredictFlowRisk(
    float flow_l_min,
    uint32_t elapsed_ms);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static const uint8_t stepper_sequence[4][4] = {
    {1U, 0U, 1U, 0U},
    {0U, 1U, 1U, 0U},
    {0U, 1U, 0U, 1U},
    {1U, 0U, 0U, 1U}
};
static void Sensors_Init(void)
{
  DWT_Delay_Init();
  DHT22_SetPinInput();
  Buzzer_Init();
  Stepper_Init();
  Relay_Init();
  flow_last_sample_tick = HAL_GetTick();
  dht_last_sample_tick = HAL_GetTick();
  telemetry_last_send_tick = HAL_GetTick();
}

static void Sensors_Service(void)
{
  Buzzer_Service();
  Stepper_Service();
  FlowSensor_Service();
  Relay_Service();
  DHT22_Service();
}

static void Stepper_Init(void)
{
  Stepper_Stop();
  stepper_last_step_tick = HAL_GetTick();
}

static void Stepper_Service(void)
{
  uint32_t now = HAL_GetTick();

  if (temperature_stepper_active == 0U)
  {
    return;
  }

  if ((now - stepper_last_step_tick) < STEPPER_STEP_INTERVAL_MS)
  {
    return;
  }

  stepper_sequence_index = (uint8_t)((stepper_sequence_index + 1U) % 4U);
  stepper_last_step_tick = now;

  HAL_GPIO_WritePin(STEPPER_IN1_GPIO_Port, STEPPER_IN1_Pin, (GPIO_PinState)stepper_sequence[stepper_sequence_index][0]);
  HAL_GPIO_WritePin(STEPPER_IN2_GPIO_Port, STEPPER_IN2_Pin, (GPIO_PinState)stepper_sequence[stepper_sequence_index][1]);
  HAL_GPIO_WritePin(STEPPER_IN3_GPIO_Port, STEPPER_IN3_Pin, (GPIO_PinState)stepper_sequence[stepper_sequence_index][2]);
  HAL_GPIO_WritePin(STEPPER_IN4_GPIO_Port, STEPPER_IN4_Pin, (GPIO_PinState)stepper_sequence[stepper_sequence_index][3]);
}

static void Stepper_Start(void)
{
  temperature_stepper_active = 1U;
  stepper_sequence_index = 0U;
  stepper_last_step_tick = HAL_GetTick();

  HAL_GPIO_WritePin(STEPPER_IN1_GPIO_Port, STEPPER_IN1_Pin, (GPIO_PinState)stepper_sequence[0][0]);
  HAL_GPIO_WritePin(STEPPER_IN2_GPIO_Port, STEPPER_IN2_Pin, (GPIO_PinState)stepper_sequence[0][1]);
  HAL_GPIO_WritePin(STEPPER_IN3_GPIO_Port, STEPPER_IN3_Pin, (GPIO_PinState)stepper_sequence[0][2]);
  HAL_GPIO_WritePin(STEPPER_IN4_GPIO_Port, STEPPER_IN4_Pin, (GPIO_PinState)stepper_sequence[0][3]);
}

static void Stepper_Stop(void)
{
  temperature_stepper_active = 0U;
  HAL_GPIO_WritePin(GPIOB, STEPPER_IN1_Pin | STEPPER_IN2_Pin | STEPPER_IN3_Pin | STEPPER_IN4_Pin, GPIO_PIN_RESET);
}

static void Buzzer_Set(uint8_t enabled)
{
  GPIO_PinState pin_state = GPIO_PIN_RESET;

  buzzer_active = enabled;

  if (enabled != 0U)
  {
    pin_state = BUZZER_ACTIVE_LEVEL;
  }
  else
  {
    pin_state = (BUZZER_ACTIVE_LEVEL == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
  }

  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, pin_state);
}

static void Buzzer_Init(void)
{
  buzzer_off_tick = 0U;
  buzzer_mold_alarm_latched = 0U;
  buzzer_frost_alarm_latched = 0U;
  buzzer_flow_alarm_latched = 0U;
  Buzzer_Set(0U);
}

static void Buzzer_Trigger(uint32_t duration_ms)
{
  Buzzer_Set(1U);
  buzzer_off_tick = HAL_GetTick() + duration_ms;
}

static void Buzzer_Service(void)
{
  uint32_t now = HAL_GetTick();

  if ((buzzer_active != 0U) && ((int32_t)(now - buzzer_off_tick) >= 0))
  {
    Buzzer_Set(0U);
  }
}

static void Relay_Set(uint8_t enabled)
{
  GPIO_PinState pin_state = GPIO_PIN_RESET;

  flow_relay_active = enabled;

  if (enabled != 0U)
  {
    pin_state = FLOW_RELAY_ACTIVE_LEVEL;
  }
  else
  {
    pin_state = (FLOW_RELAY_ACTIVE_LEVEL == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
  }

  HAL_GPIO_WritePin(FLOW_RELAY_GPIO_Port, FLOW_RELAY_Pin, pin_state);
}

static void Relay_Init(void)
{
  Relay_Set(0U);
  FrostRelay_Set(0U);
}

static void Relay_Service(void)
{
  uint8_t flow_alarm_active = 0U;

  if (flow_reference_valid == 0U)
  {
    buzzer_flow_alarm_latched = 0U;
    flow_alarm_active_state = 0U;
    return;
  }

  flow_alarm_active = (uint8_t)(
      (g_flow_l_min >= (flow_reference_l_min + FLOW_TRIGGER_DELTA_LPM)) ||
      (flow_risk_predicted_state != 0U));

  if ((flow_alarm_active != 0U) && (buzzer_flow_alarm_latched == 0U))
  {
    Buzzer_Trigger(BUZZER_BEEP_DURATION_MS);
    buzzer_flow_alarm_latched = 1U;
  }
  else if (flow_alarm_active == 0U)
  {
    buzzer_flow_alarm_latched = 0U;
  }

  flow_alarm_active_state = flow_alarm_active;

  if (flow_alarm_active != 0U)
  {
    Relay_Set(1U);
  }
  else
  {
    Relay_Set(0U);
  }
}

static void FrostRelay_Set(uint8_t enabled)
{
  GPIO_PinState pin_state = GPIO_PIN_RESET;

  frost_relay_active = enabled;

  if (enabled != 0U)
  {
    pin_state = FROST_RELAY_ACTIVE_LEVEL;
  }
  else
  {
    pin_state = (FROST_RELAY_ACTIVE_LEVEL == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
  }

  HAL_GPIO_WritePin(FROST_RELAY_GPIO_Port, FROST_RELAY_Pin, pin_state);
}

static uint8_t DHT22_IsReadingPlausible(float temperature_c, float humidity_pct)
{
  if ((temperature_c < DHT22_MIN_TEMPERATURE_C) ||
      (temperature_c > DHT22_MAX_TEMPERATURE_C) ||
      (humidity_pct < 0.0f) ||
      (humidity_pct > DHT22_MAX_HUMIDITY_PCT))
  {
    return 0U;
  }

  return 1U;
}

static uint8_t PredictMoldRisk(
    float temperature_c,
    float humidity_pct,
    uint32_t now,
    uint32_t elapsed_ms,
    float temp_delta_c,
    float humidity_delta_pct)
{
  float temp_rate_c_per_ms = 0.0f;
  float humidity_rate_pct_per_ms = 0.0f;
  float predicted_temperature_c = temperature_c;
  float predicted_humidity_pct = humidity_pct;
  uint32_t reference_age_ms = 0U;
  uint32_t prediction_horizon_ms = 0U;

  if ((dht_trend_valid == 0U) || (elapsed_ms == 0U) || (climate_reference_valid == 0U))
  {
    return 0U;
  }

  if (mold_trend_rise_streak < CLIMATE_TREND_CONFIRM_SAMPLES)
  {
    return 0U;
  }

  if ((temp_delta_c < TEMP_TREND_NOISE_FLOOR_C) ||
      (humidity_delta_pct < HUMIDITY_TREND_NOISE_FLOOR_PCT))
  {
    return 0U;
  }

  temp_rate_c_per_ms = (temperature_c - last_temperature_sample_c) / (float)elapsed_ms;
  humidity_rate_pct_per_ms = (humidity_pct - last_humidity_sample_pct) / (float)elapsed_ms;

  if ((temp_rate_c_per_ms <= 0.0f) || (humidity_rate_pct_per_ms <= 0.0f))
  {
    return 0U;
  }

  reference_age_ms = now - climate_reference_tick;
  if (reference_age_ms >= CLIMATE_EVENT_WINDOW_MS)
  {
    return 0U;
  }

  prediction_horizon_ms = CLIMATE_EVENT_WINDOW_MS - reference_age_ms;
  if (prediction_horizon_ms > MOLD_PREDICTION_LOOKAHEAD_MS)
  {
    prediction_horizon_ms = MOLD_PREDICTION_LOOKAHEAD_MS;
  }

  predicted_temperature_c += temp_rate_c_per_ms * (float)prediction_horizon_ms;
  predicted_humidity_pct += humidity_rate_pct_per_ms * (float)prediction_horizon_ms;

  return (uint8_t)(
      (predicted_temperature_c >= MOLD_TEMP_THRESHOLD_C) &&
      (predicted_humidity_pct >= MOLD_HUMIDITY_THRESHOLD_PCT) &&
      ((predicted_temperature_c - climate_reference_temperature_c) >= MOLD_TEMP_DELTA_C) &&
      ((predicted_humidity_pct - climate_reference_humidity_pct) >= MOLD_HUMIDITY_DELTA_PCT));
}

static uint8_t PredictFrostRisk(
    float temperature_c,
    uint32_t now,
    uint32_t elapsed_ms,
    float temp_delta_c)
{
  float temp_rate_c_per_ms = 0.0f;
  float predicted_temperature_c = temperature_c;
  uint32_t reference_age_ms = 0U;
  uint32_t prediction_horizon_ms = 0U;

  if ((dht_trend_valid == 0U) || (elapsed_ms == 0U) || (frost_reference_valid == 0U))
  {
    return 0U;
  }

  if (frost_trend_drop_streak < CLIMATE_TREND_CONFIRM_SAMPLES)
  {
    return 0U;
  }

  if (temp_delta_c > (-TEMP_TREND_NOISE_FLOOR_C))
  {
    return 0U;
  }

  temp_rate_c_per_ms = (temperature_c - last_temperature_sample_c) / (float)elapsed_ms;
  if (temp_rate_c_per_ms >= 0.0f)
  {
    return 0U;
  }

  reference_age_ms = now - frost_reference_tick;
  if (reference_age_ms >= CLIMATE_EVENT_WINDOW_MS)
  {
    return 0U;
  }

  prediction_horizon_ms = CLIMATE_EVENT_WINDOW_MS - reference_age_ms;
  if (prediction_horizon_ms > FROST_PREDICTION_LOOKAHEAD_MS)
  {
    prediction_horizon_ms = FROST_PREDICTION_LOOKAHEAD_MS;
  }

  predicted_temperature_c += temp_rate_c_per_ms * (float)prediction_horizon_ms;

  return (uint8_t)(
      (predicted_temperature_c <= (frost_reference_temperature_c - FROST_TRIGGER_DELTA_C)));
}

static uint8_t PredictFlowRisk(
    float flow_l_min,
    uint32_t elapsed_ms)
{
  float flow_rate_lpm_per_ms = 0.0f;
  float predicted_flow_l_min = flow_l_min;

  if ((flow_trend_valid == 0U) || (elapsed_ms == 0U) || (flow_reference_valid == 0U))
  {
    return 0U;
  }

  flow_rate_lpm_per_ms = (flow_l_min - last_flow_sample_l_min) / (float)elapsed_ms;
  if (flow_rate_lpm_per_ms <= 0.0f)
  {
    return 0U;
  }

  if (fabsf(flow_l_min - last_flow_sample_l_min) < FLOW_TREND_NOISE_FLOOR_LPM)
  {
    return 0U;
  }

  predicted_flow_l_min += flow_rate_lpm_per_ms * (float)FLOW_PREDICTION_LOOKAHEAD_MS;
  return (uint8_t)(predicted_flow_l_min >= (flow_reference_l_min + FLOW_TRIGGER_DELTA_LPM));
}

static void ESP_FlushRx(void)
{
  uint8_t byte = 0U;

  while (HAL_UART_Receive(&huart2, &byte, 1U, 2U) == HAL_OK)
  {
  }
}

static uint8_t ESP_WaitForResponse(const char *expected_ok, const char *expected_alt_ok, const char *expected_fail, uint32_t timeout_ms)
{
  uint8_t byte = 0U;
  uint32_t start_tick = HAL_GetTick();
  size_t used = 0U;

  esp_rx_line[0] = '\0';

  while ((HAL_GetTick() - start_tick) < timeout_ms)
  {
    if (HAL_UART_Receive(&huart2, &byte, 1U, 1U) == HAL_OK)
    {
      if (used < (ESP_RX_BUFFER_SIZE - 1U))
      {
        esp_rx_line[used++] = (char)byte;
      }
      else
      {
        memmove(esp_rx_line, esp_rx_line + 1, ESP_RX_BUFFER_SIZE - 2U);
        esp_rx_line[ESP_RX_BUFFER_SIZE - 2U] = (char)byte;
        used = ESP_RX_BUFFER_SIZE - 1U;
      }

      esp_rx_line[used] = '\0';

      if ((expected_ok != NULL) && (strstr(esp_rx_line, expected_ok) != NULL))
      {
        return 1U;
      }

      if ((expected_alt_ok != NULL) && (strstr(esp_rx_line, expected_alt_ok) != NULL))
      {
        return 1U;
      }

      if ((expected_fail != NULL) && (strstr(esp_rx_line, expected_fail) != NULL))
      {
        return 0U;
      }

      if ((strstr(esp_rx_line, "busy") != NULL) ||
          (strstr(esp_rx_line, "SEND FAIL") != NULL) ||
          (strstr(esp_rx_line, "link is not valid") != NULL))
      {
        return 0U;
      }
    }

    Sensors_Service();
  }

  return 0U;
}

static uint8_t ESP_SendCommandAndWait(const char *command, const char *expected_ok, const char *expected_alt_ok, const char *expected_fail, uint32_t timeout_ms)
{
  ESP_FlushRx();

  if (HAL_UART_Transmit(&huart2, (uint8_t *)command, strlen(command), 1000U) != HAL_OK)
  {
    return 0U;
  }

  return ESP_WaitForResponse(expected_ok, expected_alt_ok, expected_fail, timeout_ms);
}

static uint8_t ESP_InitModule(void)
{
  uint8_t attempt = 0U;

  for (attempt = 0U; attempt < 5U; attempt++)
  {
    if (ESP_SendCommandAndWait("AT\r\n", "OK", NULL, "ERROR", 600U) != 0U)
    {
      break;
    }

    ServiceDelay(100U);
  }

  if (attempt == 5U)
  {
    return 0U;
  }

  if (ESP_SendCommandAndWait("ATE0\r\n", "OK", NULL, "ERROR", 600U) == 0U)
  {
    return 0U;
  }

  if (ESP_SendCommandAndWait("AT+CWMODE=1\r\n", "OK", "no change", "ERROR", 600U) == 0U)
  {
    return 0U;
  }

  if (ESP_SendCommandAndWait(
          "AT+CWJAP=\"" SECURE_LINK_WIFI_SSID "\",\"" SECURE_LINK_WIFI_PASSWORD "\"\r\n",
          "WIFI GOT IP",
          "OK",
          "FAIL",
          12000U) == 0U)
  {
    return 0U;
  }

  if (ESP_SendCommandAndWait("AT+CIPMUX=0\r\n", "OK", "no change", "ERROR", 600U) == 0U)
  {
    return 0U;
  }

  return 1U;
}

static uint8_t ESP_EnsureTcpConnected(void)
{
  if (esp_tcp_connected != 0U)
  {
    return 1U;
  }

  if (ESP_SendCommandAndWait("AT+CIPSTART=\"TCP\",\"192.168.4.1\",80\r\n",
                             "CONNECT",
                             "ALREADY CONNECTED",
                             "ERROR",
                             2500U) == 0U)
  {
    return 0U;
  }

  esp_tcp_connected = 1U;
  return 1U;
}

static void ESP_RecoverLink(void)
{
  esp_tcp_connected = 0U;
  (void)ESP_SendCommandAndWait("AT+CIPCLOSE\r\n", "OK", "ERROR", "FAIL", 500U);

  if (ESP_EnsureTcpConnected() != 0U)
  {
    return;
  }

  if (ESP_InitModule() != 0U)
  {
    (void)ESP_EnsureTcpConnected();
  }
}

static uint8_t ESP_SendPayload(const char *payload)
{
  char cmd_send[32];
  size_t payload_len = strlen(payload);

  if (ESP_EnsureTcpConnected() == 0U)
  {
    return 0U;
  }

  snprintf(cmd_send, sizeof(cmd_send), "AT+CIPSEND=%lu\r\n", (unsigned long)payload_len);

  if (ESP_SendCommandAndWait(cmd_send, ">", NULL, "ERROR", 1000U) == 0U)
  {
    esp_tcp_connected = 0U;
    return 0U;
  }

  ESP_FlushRx();

  if (HAL_UART_Transmit(&huart2, (uint8_t *)payload, payload_len, 1000U) != HAL_OK)
  {
    esp_tcp_connected = 0U;
    return 0U;
  }

  if (ESP_WaitForResponse("SEND OK", NULL, "ERROR", 1200U) == 0U)
  {
    esp_tcp_connected = 0U;
    return 0U;
  }

  if (strstr(esp_rx_line, "CLOSED") != NULL)
  {
    esp_tcp_connected = 0U;
  }

  return 1U;
}

static void Telemetry_Service(void)
{
  char data[100];
  char secure_packet[SECURE_TELEMETRY_MAX_PACKET_LEN];
  uint32_t now = HAL_GetTick();

  if (climate_reference_valid == 0U)
  {
    return;
  }

  if ((now - telemetry_last_send_tick) < TELEMETRY_SEND_INTERVAL_MS)
  {
    return;
  }

  telemetry_last_send_tick = now;

  BuildSensorPayload(data, sizeof(data));
  if (SecureTelemetry_Encode(
          (const uint8_t *)data,
          strlen(data),
          SECURE_LINK_PAIR_SECRET,
          secure_packet,
          sizeof(secure_packet)) == 0U)
  {
    return;
  }

  if (ESP_SendPayload(secure_packet) == 0U)
  {
    ESP_RecoverLink();
  }
}

static void ServiceDelay(uint32_t delay_ms)
{
  uint32_t start_tick = HAL_GetTick();

  while ((HAL_GetTick() - start_tick) < delay_ms)
  {
    uint32_t remaining = delay_ms - (HAL_GetTick() - start_tick);
    uint32_t chunk = (remaining > 10U) ? 10U : remaining;

    Sensors_Service();

    if (chunk > 0U)
    {
      HAL_Delay(chunk);
    }
  }
}

static void DWT_Delay_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void DelayUs(uint32_t delay_us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t cycles = delay_us * (SystemCoreClock / 1000000U);

  while ((DWT->CYCCNT - start) < cycles)
  {
  }
}

static void DHT22_SetPinOutput(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = DHT22_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT22_DATA_GPIO_Port, &GPIO_InitStruct);
}

static void DHT22_SetPinInput(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = DHT22_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT22_DATA_GPIO_Port, &GPIO_InitStruct);
}

static uint8_t DHT22_WaitForState(GPIO_PinState state, uint32_t timeout_us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t timeout_cycles = timeout_us * (SystemCoreClock / 1000000U);

  while ((DWT->CYCCNT - start) < timeout_cycles)
  {
    if (HAL_GPIO_ReadPin(DHT22_DATA_GPIO_Port, DHT22_DATA_Pin) == state)
    {
      return 1U;
    }
  }

  return 0U;
}

static uint8_t DHT22_Read(float *temperature_c, float *humidity_pct)
{
  uint8_t data[5] = {0};
  uint8_t i = 0U;
  uint16_t raw_humidity = 0U;
  uint16_t raw_temperature = 0U;

  DHT22_SetPinOutput();
  HAL_GPIO_WritePin(DHT22_DATA_GPIO_Port, DHT22_DATA_Pin, GPIO_PIN_RESET);
  DelayUs(1200U);

  DHT22_SetPinInput();
  DelayUs(30U);

  if (DHT22_WaitForState(GPIO_PIN_RESET, 100U) == 0U)
  {
    return 0U;
  }

  if (DHT22_WaitForState(GPIO_PIN_SET, 100U) == 0U)
  {
    return 0U;
  }

  if (DHT22_WaitForState(GPIO_PIN_RESET, 100U) == 0U)
  {
    return 0U;
  }

  for (i = 0U; i < 40U; i++)
  {
    if (DHT22_WaitForState(GPIO_PIN_SET, 100U) == 0U)
    {
      return 0U;
    }

    DelayUs(40U);
    data[i / 8U] <<= 1U;

    if (HAL_GPIO_ReadPin(DHT22_DATA_GPIO_Port, DHT22_DATA_Pin) == GPIO_PIN_SET)
    {
      data[i / 8U] |= 1U;
    }

    if ((i < 39U) && (DHT22_WaitForState(GPIO_PIN_RESET, 100U) == 0U))
    {
      return 0U;
    }
  }

  if (((uint8_t)(data[0] + data[1] + data[2] + data[3])) != data[4])
  {
    return 0U;
  }

  raw_humidity = ((uint16_t)data[0] << 8) | data[1];
  raw_temperature = ((uint16_t)data[2] << 8) | data[3];

  *humidity_pct = raw_humidity / 10.0f;

  if ((raw_temperature & 0x8000U) != 0U)
  {
    raw_temperature &= 0x7FFFU;
    *temperature_c = -(raw_temperature / 10.0f);
  }
  else
  {
    *temperature_c = raw_temperature / 10.0f;
  }

  return 1U;
}

static void DHT22_Service(void)
{
  float temperature_c = 0.0f;
  float humidity_pct = 0.0f;
  float temp_delta_c = 0.0f;
  float humidity_delta_pct = 0.0f;
  uint8_t mold_risk = 0U;
  uint8_t frost_risk = 0U;
  uint8_t mold_risk_predicted = 0U;
  uint8_t frost_risk_predicted = 0U;
  uint32_t trend_elapsed_ms = 0U;
  uint32_t now = HAL_GetTick();

  if ((now - dht_last_sample_tick) < DHT22_SAMPLE_INTERVAL_MS)
  {
    return;
  }

  dht_last_sample_tick = now;

  if (DHT22_Read(&temperature_c, &humidity_pct) != 0U)
  {
    if (DHT22_IsReadingPlausible(temperature_c, humidity_pct) == 0U)
    {
      g_dht_valid = 0U;
      Stepper_Stop();
      FrostRelay_Set(0U);
      mold_alarm_active = 0U;
      frost_alarm_active = 0U;
      dht_trend_valid = 0U;
      mold_trend_rise_streak = 0U;
      frost_trend_drop_streak = 0U;
      return;
    }

    if ((dht_trend_valid != 0U) && (now > last_dht_trend_tick))
    {
      trend_elapsed_ms = now - last_dht_trend_tick;
      temp_delta_c = temperature_c - last_temperature_sample_c;
      humidity_delta_pct = humidity_pct - last_humidity_sample_pct;

      if ((temp_delta_c >= TEMP_TREND_NOISE_FLOOR_C) &&
          (humidity_delta_pct >= HUMIDITY_TREND_NOISE_FLOOR_PCT))
      {
        if (mold_trend_rise_streak < 255U)
        {
          mold_trend_rise_streak++;
        }
      }
      else
      {
        mold_trend_rise_streak = 0U;
      }

      if (temp_delta_c <= (-TEMP_TREND_NOISE_FLOOR_C))
      {
        if (frost_trend_drop_streak < 255U)
        {
          frost_trend_drop_streak++;
        }
      }
      else
      {
        frost_trend_drop_streak = 0U;
      }
    }
    else
    {
      mold_trend_rise_streak = 0U;
      frost_trend_drop_streak = 0U;
    }

    g_temperature_c = temperature_c;
    g_humidity_pct = humidity_pct;
    g_dht_valid = 1U;

    if (climate_reference_valid == 0U)
    {
      climate_reference_temperature_c = temperature_c;
      climate_reference_humidity_pct = humidity_pct;
      climate_reference_tick = now;
      climate_reference_valid = 1U;
      frost_reference_temperature_c = temperature_c;
      frost_reference_tick = now;
      frost_reference_valid = 1U;
      telemetry_last_send_tick = now - TELEMETRY_SEND_INTERVAL_MS;
    }

    if (((now - climate_reference_tick) > CLIMATE_EVENT_WINDOW_MS) ||
        (temperature_c < climate_reference_temperature_c) ||
        (humidity_pct < climate_reference_humidity_pct))
    {
      climate_reference_temperature_c = temperature_c;
      climate_reference_humidity_pct = humidity_pct;
      climate_reference_tick = now;
    }

    if ((frost_reference_valid == 0U) ||
        ((now - frost_reference_tick) > CLIMATE_EVENT_WINDOW_MS) ||
        (temperature_c > frost_reference_temperature_c))
    {
      frost_reference_temperature_c = temperature_c;
      frost_reference_tick = now;
      frost_reference_valid = 1U;
    }

    mold_risk = (uint8_t)(
        (temperature_c >= MOLD_TEMP_THRESHOLD_C) &&
        (humidity_pct >= MOLD_HUMIDITY_THRESHOLD_PCT));

    frost_risk = (uint8_t)(
        (frost_reference_valid != 0U) &&
        ((now - frost_reference_tick) <= CLIMATE_EVENT_WINDOW_MS) &&
        (temperature_c <= (frost_reference_temperature_c - FROST_TRIGGER_DELTA_C)) &&
        (temp_delta_c <= (-TEMP_TREND_NOISE_FLOOR_C)));

    mold_risk_predicted = PredictMoldRisk(
        temperature_c,
        humidity_pct,
        now,
        trend_elapsed_ms,
        temp_delta_c,
        humidity_delta_pct);
    frost_risk_predicted = PredictFrostRisk(
        temperature_c,
        now,
        trend_elapsed_ms,
        temp_delta_c);

    if (((mold_risk != 0U) || (mold_risk_predicted != 0U)) && (buzzer_mold_alarm_latched == 0U))
    {
      Buzzer_Trigger(BUZZER_BEEP_DURATION_MS);
      buzzer_mold_alarm_latched = 1U;
    }
    else if ((mold_risk == 0U) && (mold_risk_predicted == 0U))
    {
      buzzer_mold_alarm_latched = 0U;
    }

    if (((frost_risk != 0U) || (frost_risk_predicted != 0U)) && (buzzer_frost_alarm_latched == 0U))
    {
      Buzzer_Trigger(BUZZER_BEEP_DURATION_MS);
      buzzer_frost_alarm_latched = 1U;
    }
    else if ((frost_risk == 0U) && (frost_risk_predicted == 0U))
    {
      buzzer_frost_alarm_latched = 0U;
    }

    if ((mold_risk != 0U) || (mold_risk_predicted != 0U))
    {
      Stepper_Start();
    }
    else
    {
      Stepper_Stop();
    }

    FrostRelay_Set((uint8_t)((frost_risk != 0U) || (frost_risk_predicted != 0U)));
    mold_alarm_active = (uint8_t)((mold_risk != 0U) || (mold_risk_predicted != 0U));
    frost_alarm_active = (uint8_t)((frost_risk != 0U) || (frost_risk_predicted != 0U));

    last_temperature_sample_c = temperature_c;
    last_humidity_sample_pct = humidity_pct;
    last_dht_trend_tick = now;
    dht_trend_valid = 1U;
  }
  else
  {
    g_dht_valid = 0U;
    Stepper_Stop();
    FrostRelay_Set(0U);
    mold_alarm_active = 0U;
    frost_alarm_active = 0U;
    dht_trend_valid = 0U;
    buzzer_mold_alarm_latched = 0U;
    buzzer_frost_alarm_latched = 0U;
    mold_trend_rise_streak = 0U;
    frost_trend_drop_streak = 0U;
  }
}

static void FlowSensor_Service(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t elapsed_ms = now - flow_last_sample_tick;
  uint32_t pulse_delta = 0U;
  float frequency_hz = 0.0f;
  float flow_l_min = 0.0f;

  if (elapsed_ms < FLOW_SAMPLE_INTERVAL_MS)
  {
    return;
  }

  pulse_delta = g_flow_pulse_count - flow_last_sample_pulses;
  frequency_hz = (pulse_delta * 1000.0f) / elapsed_ms;

  if (pulse_delta == 0U)
  {
    flow_l_min = 0.0f;
  }
  else
  {
    flow_l_min = (frequency_hz + 3.0f) / 23.0f;
  }

  g_flow_frequency_hz = frequency_hz;
  g_flow_l_min = flow_l_min;
  g_flow_total_l = g_flow_pulse_count / 1380.0f;

  if (flow_reference_valid == 0U)
  {
    flow_reference_l_min = flow_l_min;
    flow_reference_valid = 1U;
  }

  if ((flow_trend_valid != 0U) && (now > last_flow_trend_tick))
  {
    flow_risk_predicted_state = PredictFlowRisk(flow_l_min, now - last_flow_trend_tick);
  }
  else
  {
    flow_risk_predicted_state = 0U;
  }

  last_flow_sample_l_min = flow_l_min;
  last_flow_trend_tick = now;
  flow_trend_valid = 1U;

  flow_last_sample_tick = now;
  flow_last_sample_pulses = g_flow_pulse_count;
}

static void BuildSensorPayload(char *buffer, size_t buffer_len)
{
  int32_t temperature_x10 = (int32_t)(g_temperature_c * 10.0f);
  uint32_t temperature_fraction = (uint32_t)abs((int)(temperature_x10 % 10));
  uint32_t humidity_x10 = (uint32_t)(g_humidity_pct * 10.0f);
  uint32_t flow_x100 = (uint32_t)(g_flow_l_min * 100.0f);

  snprintf(
      buffer,
      buffer_len,
      "Sicaklik: %ld.%luC, Nem: %lu.%lu%%, Debi: %lu.%02luL/dk, Mantar: %u, Don: %u, Boru: %u\r\n",
      (long)(temperature_x10 / 10),
      (unsigned long)temperature_fraction,
      (unsigned long)(humidity_x10 / 10U),
      (unsigned long)(humidity_x10 % 10U),
      (unsigned long)(flow_x100 / 100U),
      (unsigned long)(flow_x100 % 100U),
      (unsigned int)(mold_alarm_active != 0U),
      (unsigned int)(frost_alarm_active != 0U),
      (unsigned int)(flow_alarm_active_state != 0U));
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  Sensors_Init();
  ServiceDelay(500U);
  (void)ESP_InitModule();
  (void)ESP_EnsureTcpConnected();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Sensors_Service();
    Telemetry_Service();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(FLOW_RELAY_GPIO_Port, FLOW_RELAY_Pin,
                    (FLOW_RELAY_ACTIVE_LEVEL == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);
  HAL_GPIO_WritePin(FROST_RELAY_GPIO_Port, FROST_RELAY_Pin,
                    (FROST_RELAY_ACTIVE_LEVEL == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,
                    (BUZZER_ACTIVE_LEVEL == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB5 PB6 PB7
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_GPIO_WritePin(FLOW_RELAY_GPIO_Port, FLOW_RELAY_Pin,
                    (FLOW_RELAY_ACTIVE_LEVEL == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);
  HAL_GPIO_WritePin(FROST_RELAY_GPIO_Port, FROST_RELAY_Pin,
                    (FROST_RELAY_ACTIVE_LEVEL == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);
  GPIO_InitStruct.Pin = STEPPER_IN1_Pin | STEPPER_IN2_Pin | STEPPER_IN3_Pin | STEPPER_IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOB, STEPPER_IN1_Pin | STEPPER_IN2_Pin | STEPPER_IN3_Pin | STEPPER_IN4_Pin, GPIO_PIN_RESET);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == FLOW_SENSOR_Pin)
  {
    g_flow_pulse_count++;
  }
}

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
