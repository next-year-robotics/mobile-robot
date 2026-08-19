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

/* 최악의 줄: "ticks " 6 + uint32 10 + ' ' + int64 20 + ' ' + int64 20 + '\n' = 58 */
#define TICKS_LINE_CAP            64U

/* 로봇 전진 시 좌우 누적 틱이 모두 +가 되게 하는 보정. 2026-08-20 M2 실측:
   왼쪽 원시 부호 +, 오른쪽 원시 부호 -. 두 바퀴가 마주 보게 달려 있어 생기는
   차이이므로 배선이 아니라 여기서 잡는다. */
#define ENCODER_SIGN_LEFT         (+1)
#define ENCODER_SIGN_RIGHT        (-1)

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
  * @brief  "ticks <ms> <left> <right>\n"을 buf 뒤에서부터 채운다.
  * @retval 줄의 시작 위치. 길이는 out_length로 돌려준다.
  */
static char *build_ticks_line(char *buf, size_t capacity, uint32_t now_ms,
                              int64_t left, int64_t right, size_t *out_length)
{
  static const char tag[] = "ticks ";
  char *p = buf + capacity;

  *--p = '\n';
  p = write_i64(p, right);
  *--p = ' ';
  p = write_i64(p, left);
  *--p = ' ';
  p = write_i64(p, (int64_t)now_ms);

  p -= sizeof(tag) - 1U;
  memcpy(p, tag, sizeof(tag) - 1U);

  *out_length = (size_t)((buf + capacity) - p);
  return p;
}

/**
  * @brief  좌우 카운터를 읽어 델타를 누적한다.
  * @note   (int16_t) 캐스팅이 16-bit 랩어라운드를 처리하므로 오버플로를 따로
  *         추적하지 않는다. 손으로 돌리는 속도에서 10 ms 델타는 ±32767 근처에
  *         가지 않는다.
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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  char tx_buffer[TICKS_LINE_CAP];
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
  /* USER CODE BEGIN 2 */
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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now_ms = HAL_GetTick();

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
      char *line;
      size_t length;

      last_report_ms += TICKS_REPORT_PERIOD_MS;
      line = build_ticks_line(tx_buffer, sizeof(tx_buffer), now_ms,
                              g_ticks_left, g_ticks_right, &length);

      if (HAL_UART_Transmit(&huart2, (const uint8_t *)line, (uint16_t)length,
                            HAL_MAX_DELAY) != HAL_OK)
      {
        Error_Handler();
      }
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
