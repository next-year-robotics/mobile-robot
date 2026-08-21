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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LD2_TOGGLE_PERIOD_MS     500U
#define ENCODER_SAMPLE_PERIOD_MS  10U
#define TICKS_REPORT_PERIOD_MS    20U  /* 50 Hz */

/* 최악의 줄: "ticks" 5 + (' '+uint32 10) + (' '+int64 20) x 2 + '\n' = 58 */
#define TX_LINE_CAP               64U

/* 로봇 전진 시 좌우 누적 틱이 모두 +가 되게 하는 보정. 2026-08-20 M2 실측:
   왼쪽 원시 부호 +, 오른쪽 원시 부호 -. 두 바퀴가 마주 보게 달려 있어 생기는
   차이이므로 배선이 아니라 여기서 잡는다. */
#define ENCODER_SIGN_LEFT         (+1)
#define ENCODER_SIGN_RIGHT        (-1)

/* ---- M3 모터 ---- */

/* TIM1 ARR. .ioc의 TIM1.Period와 반드시 같아야 하며 main()에서 대조한다. */
#define MOTOR_PWM_ARR             4666U

/* duty 지령 단위는 per-mille 정수다. MCU에서 부동소수 파싱을 하지 않는다. */
#define MOTOR_DUTY_MAX_PM         1000

/* 양수 duty = 로봇 전진이 되게 하는 보정. 두 바퀴가 마주 보게 달려 있어 한쪽은
   -1이 될 가능성이 높다. M3.md 5.3에서 실측해 확정한다. 그때까지 잠정값이다. */
#define MOTOR_SIGN_LEFT           (-1)
#define MOTOR_SIGN_RIGHT          (+1)

/* 마지막 유효 명령으로부터 이 시간이 지나면 duty 0. 호스트 툴이 죽거나 USB가
   빠져도 모터가 계속 돌지 않게 한다. limits.md의 확정값 200 ms는 /cmd_vel 주기
   기준이며 M4/M5에서 적용한다. M3는 사람이 개입하는 수동 시험이라 500 ms다. */
#define CMD_WATCHDOG_MS           500U

#define CMD_RX_RING_SIZE          64U  /* 2의 거듭제곱이어야 한다 */
#define CMD_LINE_CAP              32U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 로봇 전진 = + 로 보정된 누적 틱. 보정값은 ENCODER_SIGN_* 참고. */
static int64_t g_ticks_left;
static int64_t g_ticks_right;

/* 직전 샘플의 카운터. TIM2는 32-bit 하드웨어지만 ARR=65535라 16-bit로 감긴다. */
static uint16_t g_prev_left;
static uint16_t g_prev_right;

/* 송신 버퍼. 송신은 전부 main 루프에서만 하므로 하나를 돌려 쓴다. */
static char g_tx_buffer[TX_LINE_CAP];

/* 호스트가 지령한 duty(per-mille). 워치독이 만료돼도 이 값은 남고 출력만 0이 된다. */
static int32_t g_cmd_duty_left;
static int32_t g_cmd_duty_right;
static uint32_t g_last_cmd_ms;
static uint8_t g_watchdog_tripped;
static uint8_t g_watchdog_notify;

/* USART2 수신 링버퍼. head는 ISR이, tail은 main이 움직인다. */
static volatile uint8_t g_rx_ring[CMD_RX_RING_SIZE];
static volatile uint16_t g_rx_head;
static volatile uint16_t g_rx_tail;
static uint8_t g_rx_byte;

/* 조립 중인 명령 줄. */
static char g_line[CMD_LINE_CAP];
static size_t g_line_length;
static uint8_t g_line_overflow;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  value를 p 바로 앞쪽에 10진수로 쓰고 새 시작 위치를 반환한다.
  * @note   링커가 nano.specs를 쓰므로 printf 계열은 %lld를 다루지 못한다.
  *         64-bit 누적 틱은 직접 변환한다.
  */
static char *write_i64(char *p, int64_t value)
{
  /* INT64_MIN도 부호 반전 없이 다루도록 크기는 부호 없는 타입으로 잡는다. */
  uint64_t magnitude = (value < 0) ? (~(uint64_t)value + 1U) : (uint64_t)value;

  do
  {
    *--p = (char)('0' + (uint32_t)(magnitude % 10U));
    magnitude /= 10U;
  } while (magnitude != 0U);

  if (value < 0)
  {
    *--p = '-';
  }

  return p;
}

/**
  * @brief  "<tag> <v0> <v1> ...\n"을 buf 뒤에서부터 채운다.
  * @retval 줄의 시작 위치. 길이는 out_length로 돌려준다.
  */
static char *build_line(char *buf, size_t capacity, const char *tag,
                        const int64_t *values, size_t count, size_t *out_length)
{
  size_t tag_length = strlen(tag);
  char *p = buf + capacity;
  size_t i = count;

  *--p = '\n';
  while (i-- > 0U)
  {
    p = write_i64(p, values[i]);
    *--p = ' ';
  }

  p -= tag_length;
  memcpy(p, tag, tag_length);

  *out_length = (size_t)((buf + capacity) - p);
  return p;
}

/**
  * @brief  한 줄을 USART2로 내보낸다.
  * @note   송신은 전부 main 루프에서만 한다. ISR에서는 부르지 않는다.
  */
static void uart_send_line(const char *tag, const int64_t *values, size_t count)
{
  size_t length;
  char *line = build_line(g_tx_buffer, sizeof(g_tx_buffer), tag, values, count,
                          &length);

  if (HAL_UART_Transmit(&huart2, (const uint8_t *)line, (uint16_t)length,
                        HAL_MAX_DELAY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  좌우 카운터를 읽어 델타를 누적한다.
  * @note   (int16_t) 캐스팅이 16-bit 랩어라운드를 처리하므로 오버플로를 따로
  *         추적하지 않는다. 100 Hz에서 최대 틱레이트를 넣어도 델타는 ±32767
  *         근처에 가지 않는다.
  */
static void encoder_sample(void)
{
  uint16_t now_left = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  uint16_t now_right = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);

  g_ticks_left += ENCODER_SIGN_LEFT * (int16_t)(now_left - g_prev_left);
  g_ticks_right += ENCODER_SIGN_RIGHT * (int16_t)(now_right - g_prev_right);

  g_prev_left = now_left;
  g_prev_right = now_right;
}

/**
  * @brief  한 채널의 두 입력에 sign-magnitude로 duty를 건다.
  * @note   MDD3A는 한쪽에 PWM, 다른 쪽에 0을 넣어 방향을 정한다. 둘 다 High는
  *         브레이크지만 이번 범위에서 쓰지 않는다. 반대쪽을 먼저 0으로 만든 뒤
  *         이쪽을 올리므로 두 입력이 동시에 0이 아닌 순간이 생기지 않는다.
  */
static void motor_apply(volatile uint32_t *ccr_a, volatile uint32_t *ccr_b,
                        int32_t duty_pm)
{
  uint32_t pulse;

  if (duty_pm > MOTOR_DUTY_MAX_PM)
  {
    duty_pm = MOTOR_DUTY_MAX_PM;
  }
  else if (duty_pm < -MOTOR_DUTY_MAX_PM)
  {
    duty_pm = -MOTOR_DUTY_MAX_PM;
  }
  else
  {
    /* 범위 안 */
  }

  /* |duty| x (ARR+1) / 1000. duty=1000이면 CCR=4667 > ARR이라 100 % 온이다. */
  pulse = (uint32_t)(((duty_pm < 0) ? -duty_pm : duty_pm)
                     * (int32_t)(MOTOR_PWM_ARR + 1U) / MOTOR_DUTY_MAX_PM);

  if (duty_pm >= 0)
  {
    *ccr_b = 0U;
    *ccr_a = pulse;
  }
  else
  {
    *ccr_a = 0U;
    *ccr_b = pulse;
  }
}

/**
  * @brief  좌우 duty를 TIM1 네 채널에 반영한다.
  * @note   CCR을 만지는 유일한 정상 경로다(강제 정지는 Error_Handler 참고).
  *         PA8=CH1=M1A, PA9=CH2=M1B, PA10=CH3=M2A, PA11=CH4=M2B.
  */
static void motor_set_duty(int32_t left_pm, int32_t right_pm)
{
  motor_apply(&TIM1->CCR1, &TIM1->CCR2, MOTOR_SIGN_LEFT * left_pm);
  motor_apply(&TIM1->CCR3, &TIM1->CCR4, MOTOR_SIGN_RIGHT * right_pm);
}

/**
  * @brief  워치독을 반영해 실제 CCR을 매 루프 다시 쓴다.
  * @note   지령은 기억해 두고 출력은 매번 새로 계산한다. 이러면 "지난 duty가
  *         어딘가에 남아 있는" 경로 자체가 존재할 수 없다.
  */
static void motor_update(uint32_t now_ms)
{
  if ((uint32_t)(now_ms - g_last_cmd_ms) >= CMD_WATCHDOG_MS)
  {
    if (g_watchdog_tripped == 0U)
    {
      g_watchdog_tripped = 1U;
      g_watchdog_notify = 1U;
    }
    motor_set_duty(0, 0);
  }
  else
  {
    g_watchdog_tripped = 0U;
    motor_set_duty(g_cmd_duty_left, g_cmd_duty_right);
  }
}

/**
  * @brief  공백을 건너뛰고 10진 정수 하나를 읽는다.
  * @retval 성공 시 1, 숫자가 없거나 자릿수가 넘치면 0.
  */
static int parse_i32(const char **cursor, int32_t *out)
{
  const char *p = *cursor;
  int32_t value = 0;
  int32_t sign = 1;
  int digits = 0;

  while (*p == ' ')
  {
    p++;
  }

  if (*p == '-')
  {
    sign = -1;
    p++;
  }
  else if (*p == '+')
  {
    p++;
  }
  else
  {
    /* 부호 없음 */
  }

  while ((*p >= '0') && (*p <= '9'))
  {
    /* per-mille 지령이라 여섯 자리를 넘길 일이 없다. 넘치면 실패로 본다. */
    if (digits >= 6)
    {
      return 0;
    }
    value = (value * 10) + (int32_t)(*p - '0');
    p++;
    digits++;
  }

  if (digits == 0)
  {
    return 0;
  }

  *out = sign * value;
  *cursor = p;
  return 1;
}

/**
  * @brief  완성된 한 줄을 해석하고 응답한다.
  * @note   파싱에 실패한 줄은 명령으로 치지 않는다 — g_last_cmd_ms를 갱신하지
  *         않으므로 호스트가 쓰레기를 보내는 동안 워치독이 계속 흐른다.
  */
static void command_execute(const char *line, uint32_t now_ms)
{
  const char *p = line;
  int64_t values[3];
  int32_t left = 0;
  int32_t right = 0;
  int accepted = 0;

  if ((strncmp(p, "duty", 4) == 0) && ((p[4] == ' ') || (p[4] == '\0')))
  {
    p += 4;
    if ((parse_i32(&p, &left) != 0) && (parse_i32(&p, &right) != 0))
    {
      while (*p == ' ')
      {
        p++;
      }
      accepted = (*p == '\0') ? 1 : 0;
    }
  }
  else if (strcmp(p, "stop") == 0)
  {
    accepted = 1;
  }
  else
  {
    /* 모르는 명령 */
  }

  if (accepted == 0)
  {
    values[0] = (int64_t)now_ms;
    uart_send_line("err", values, 1U);
    return;
  }

  /* 실제 출력은 어차피 클램프된다. 호스트가 무엇이 적용됐는지 알 수 있도록
     ack에는 클램프 뒤의 값을 싣는다. */
  if (left > MOTOR_DUTY_MAX_PM)
  {
    left = MOTOR_DUTY_MAX_PM;
  }
  else if (left < -MOTOR_DUTY_MAX_PM)
  {
    left = -MOTOR_DUTY_MAX_PM;
  }
  else
  {
    /* 범위 안 */
  }

  if (right > MOTOR_DUTY_MAX_PM)
  {
    right = MOTOR_DUTY_MAX_PM;
  }
  else if (right < -MOTOR_DUTY_MAX_PM)
  {
    right = -MOTOR_DUTY_MAX_PM;
  }
  else
  {
    /* 범위 안 */
  }

  g_cmd_duty_left = left;
  g_cmd_duty_right = right;
  g_last_cmd_ms = now_ms;

  values[0] = (int64_t)now_ms;
  values[1] = (int64_t)left;
  values[2] = (int64_t)right;
  uart_send_line("ok", values, 3U);
}

/**
  * @brief  수신 링버퍼를 비우며 줄이 완성될 때마다 해석한다.
  */
static void command_poll(uint32_t now_ms)
{
  int64_t value;

  while (g_rx_tail != g_rx_head)
  {
    char c = (char)g_rx_ring[g_rx_tail];

    g_rx_tail = (uint16_t)((g_rx_tail + 1U) & (CMD_RX_RING_SIZE - 1U));

    if ((c == '\n') || (c == '\r'))
    {
      if (g_line_length == 0U)
      {
        continue;  /* 빈 줄과 CRLF의 뒤쪽 문자를 무시한다 */
      }

      if (g_line_overflow != 0U)
      {
        g_line_overflow = 0U;
        g_line_length = 0U;
        value = (int64_t)now_ms;
        uart_send_line("err", &value, 1U);
        continue;
      }

      g_line[g_line_length] = '\0';
      g_line_length = 0U;
      command_execute(g_line, now_ms);
      continue;
    }

    if (g_line_length >= (CMD_LINE_CAP - 1U))
    {
      g_line_overflow = 1U;  /* 줄이 끝날 때까지 버리고 err로 답한다 */
      continue;
    }

    g_line[g_line_length] = c;
    g_line_length++;
  }
}

/**
  * @brief  USART2 수신 완료. 한 바이트를 링버퍼에 넣고 즉시 다시 건다.
  * @note   ISR이다. 송신도 snprintf도 하지 않는다.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uint16_t next = (uint16_t)((g_rx_head + 1U) & (CMD_RX_RING_SIZE - 1U));

    /* 가득 차면 버린다. ISR에서 기다리지 않는다. */
    if (next != g_rx_tail)
    {
      g_rx_ring[g_rx_head] = g_rx_byte;
      g_rx_head = next;
    }

    (void)HAL_UART_Receive_IT(huart, &g_rx_byte, 1U);
  }
}

/**
  * @brief  수신 오류(주로 오버런). HAL이 수신을 끊어 놓으므로 다시 건다.
  * @note   흐름제어가 없는 링크라 오버런은 실제로 일어난다. 여기서 되살리지
  *         않으면 그 뒤로 명령이 영영 들어오지 않는다.
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    (void)HAL_UART_Receive_IT(huart, &g_rx_byte, 1U);
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint32_t last_led_toggle_ms;
  uint32_t last_sample_ms;
  uint32_t last_report_ms;
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  /* .ioc의 TIM1.Period가 바뀌면 duty 환산이 조용히 틀어진다. 먼저 대조한다. */
  if (htim1.Init.Period != MOTOR_PWM_ARR)
  {
    Error_Handler();
  }

  /* MOE가 서기 전에 네 CCR을 0으로 확정한다. 출력이 살아나는 순간의 duty가 0이다. */
  motor_set_duty(0, 0);

  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Error_Handler();
  }

  /* 시작 시점의 카운터를 기준으로 잡아 첫 델타가 0이 되게 한다. */
  g_prev_left = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  g_prev_right = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);

  last_led_toggle_ms = HAL_GetTick();
  last_sample_ms = last_led_toggle_ms;
  last_report_ms = last_led_toggle_ms;

  /* 첫 명령을 받기 전에는 워치독이 이미 만료된 상태로 둔다. 부호 없는 뺄셈이라
     지금 시각에서 만료 시간을 빼두면 (now - g_last_cmd_ms)가 곧바로 만료로
     계산된다. 부팅 직후 wd 한 줄이 뜨는 것을 막으면서 출력은 0으로 묶인다. */
  g_last_cmd_ms = last_led_toggle_ms - CMD_WATCHDOG_MS;
  g_watchdog_tripped = 1U;

  if (HAL_UART_Receive_IT(&huart2, &g_rx_byte, 1U) != HAL_OK)
  {
    Error_Handler();
  }

/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now_ms = HAL_GetTick();

    /* 오류로 수신이 풀려 있으면 다시 건다. ErrorCallback이 놓친 경우의 그물이다. */
    if (huart2.RxState != HAL_UART_STATE_BUSY_RX)
    {
      (void)HAL_UART_Receive_IT(&huart2, &g_rx_byte, 1U);
    }

    command_poll(now_ms);
    motor_update(now_ms);

    if (g_watchdog_notify != 0U)
    {
      int64_t value = (int64_t)now_ms;

      g_watchdog_notify = 0U;
      uart_send_line("wd", &value, 1U);
    }

    if ((uint32_t)(now_ms - last_led_toggle_ms) >= LD2_TOGGLE_PERIOD_MS)
    {
      last_led_toggle_ms += LD2_TOGGLE_PERIOD_MS;
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }

    if ((uint32_t)(now_ms - last_sample_ms) >= ENCODER_SAMPLE_PERIOD_MS)
    {
      last_sample_ms += ENCODER_SAMPLE_PERIOD_MS;
      encoder_sample();
    }

    if ((uint32_t)(now_ms - last_report_ms) >= TICKS_REPORT_PERIOD_MS)
    {
      int64_t values[3];

      last_report_ms += TICKS_REPORT_PERIOD_MS;
      values[0] = (int64_t)now_ms;
      values[1] = g_ticks_left;
      values[2] = g_ticks_right;
      uart_send_line("ticks", values, 3U);
    }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  /* 여기서 멈추는 순간 모터가 돌고 있을 수 있다. 소프트웨어 워치독은 main 루프가
     돌아야 동작하므로 무한 루프에 들어가기 전에 직접 끈다. MOE를 내리면 TIM1
     출력이 해제되고, MDD3A 입력은 10 kΩ 풀다운이 Low로 잡는다. */
  TIM1->CCR1 = 0U;
  TIM1->CCR2 = 0U;
  TIM1->CCR3 = 0U;
  TIM1->CCR4 = 0U;
  TIM1->BDTR &= ~TIM_BDTR_MOE;

  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
