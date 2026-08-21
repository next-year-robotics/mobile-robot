/**
  ******************************************************************************
  * @file    platform_stm32.c
  * @brief   STM32F446 adapter. HAL 레지스터 기록은 이 파일 안에만 존재한다.
  ******************************************************************************
  */
#include "platform_stm32.h"

#include "app_config.h"
#include "app_main.h"

#include "iwdg.h"
#include "main.h"
#include "tim.h"
#include "usart.h"

/* USART2 수신 인터럽트가 한 바이트씩 채우는 자리. */
static uint8_t s_rx_byte;
static uint32_t s_boot_reset_flags;

void platform_boot_snapshot_reset_flags(void)
{
  const uint32_t reset_mask = RCC_CSR_LPWRRSTF | RCC_CSR_WWDGRSTF |
                              RCC_CSR_IWDGRSTF | RCC_CSR_SFTRSTF |
                              RCC_CSR_PORRSTF | RCC_CSR_PINRSTF |
                              RCC_CSR_BORRSTF;

  s_boot_reset_flags = RCC->CSR & reset_mask;
  __HAL_RCC_CLEAR_RESET_FLAGS();
}

uint32_t platform_boot_reset_flags(void)
{
  return s_boot_reset_flags;
}

bool platform_boot_was_iwdg_reset(void)
{
  return (s_boot_reset_flags & RCC_CSR_IWDGRSTF) != 0U;
}

bool platform_iwdg_refresh(void)
{
  return HAL_IWDG_Refresh(&hiwdg) == HAL_OK;
}

/**
  * @brief  한 채널의 두 CCR을 안전한 순서로 기록한다.
  */
static void write_pair(volatile uint32_t *reg_a, volatile uint32_t *reg_b,
                       uint32_t value_a, uint32_t value_b)
{
  if (value_a == 0U)
  {
    *reg_a = 0U;
    *reg_b = value_b;
  }
  else
  {
    *reg_b = 0U;
    *reg_a = value_a;
  }
}

void platform_motor_write(const motor_ccr_t *out)
{
  uint32_t state = platform_critical_enter();

  write_pair(&TIM1->CCR1, &TIM1->CCR2,
             out->ccr[MOTOR_CCR_LEFT_A], out->ccr[MOTOR_CCR_LEFT_B]);
  write_pair(&TIM1->CCR3, &TIM1->CCR4,
             out->ccr[MOTOR_CCR_RIGHT_A], out->ccr[MOTOR_CCR_RIGHT_B]);
  platform_critical_exit(state);
}

/* 남은 call site는 platform_motor_kill() 내부 하나뿐이다(2026-08-21 감사).
   RX 손상 ISR이 여기를 직접 부르던 경로는 urgent STOP 이벤트로 대체했다. */
void platform_motor_zero(void)
{
  TIM1->CCR1 = 0U;
  TIM1->CCR2 = 0U;
  TIM1->CCR3 = 0U;
  TIM1->CCR4 = 0U;
}

void platform_motor_kill(void)
{
  /* 소프트웨어 워치독은 super-loop가 돌아야 동작한다. 여기로 오는 경로는 그게
     보장되지 않으므로 TIM1 출력을 직접 끊는다. MOE를 내리면 TIM1 출력이 해제되고
     MDD3A 입력은 10 kOhm 풀다운이 Low로 잡는다.

     치명 경로에서는 핀 차단이 최우선이므로 MOE를 먼저 내리고 CCR을 지운다.
     BDTR의 read-modify-write는 원자적이지 않지만, 초기화 이후 이 경로는 MOE를
     내리기만 한다. 락도 할당도 HAL 블로킹 API도 쓰지 않는다. */
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  platform_motor_zero();
}

uint32_t platform_now_ms(void)
{
  return HAL_GetTick();
}

uint32_t platform_dwt_cycles(void)
{
  return DWT->CYCCNT;
}

/**
  * @brief  DWT cycle counter를 켠다. scheduler 전에 한 번만 부른다.
  * @note   Cortex-M4에는 CM7의 DWT lock access register가 없다. TRCENA만 세우면 된다.
  */
static void platform_dwt_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

bool platform_control_timer_start(void)
{
  return HAL_TIM_Base_Start_IT(&htim6) == HAL_OK;
}

uint16_t platform_encoder_left_count(void)
{
  return (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
}

uint16_t platform_encoder_right_count(void)
{
  return (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
}

bool platform_uart_send(const char *data, size_t length)
{
  return HAL_UART_Transmit(&huart2, (const uint8_t *)data, (uint16_t)length,
                           UART_TX_TIMEOUT_MS) == HAL_OK;
}

bool platform_uart_rx_is_armed(void)
{
  return huart2.RxState == HAL_UART_STATE_BUSY_RX;
}

bool platform_uart_rx_arm(void)
{
  return HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U) == HAL_OK;
}

uint32_t platform_critical_enter(void)
{
  uint32_t state = __get_PRIMASK();

  __disable_irq();
  __DMB();
  return state;
}

void platform_critical_exit(uint32_t state)
{
  __DMB();
  if ((state & 1U) == 0U)
  {
    __enable_irq();
  }
}

void platform_led_toggle(void)
{
  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}

bool platform_init(void)
{
  motor_ccr_t zero = motor_ccr_from_duty(0, 0);

  /* .ioc의 TIM1.Period가 바뀌면 duty 환산이 조용히 틀어진다. 먼저 대조한다. */
  if (htim1.Init.Period != MOTOR_PWM_ARR)
  {
    return false;
  }

  /* MOE가 서기 전에 네 CCR을 0으로 확정한다. 출력이 살아나는 순간의 duty가 0이다. */
  platform_motor_write(&zero);

  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    return false;
  }
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
  {
    return false;
  }
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
  {
    return false;
  }
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
  {
    return false;
  }

  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
  {
    return false;
  }
  if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
  {
    return false;
  }

  /* control WCET/wake latency 계측의 시간 기준. scheduler 전에 한 번만 켠다. */
  platform_dwt_init();

  /* USART2 수신 무장과 TIM6 기동은 여기서 하지 않는다. 두 인터럽트 모두 task를
     깨우므로 handle과 static 객체가 생긴 뒤, 각 소유 task의 첫머리에서 시작한다. */
  return true;
}

/**
  * @brief  USART2 수신 완료. 한 바이트를 링에 넣고 즉시 다시 건다.
  * @note   ISR이다. 송신도 snprintf도 하지 않는다 (M2 규칙 유지).
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uint32_t pending_error = huart->ErrorCode;

    app_on_rx_byte(s_rx_byte);

    if (HAL_UART_Receive_IT(huart, &s_rx_byte, 1U) != HAL_OK)
    {
      app_on_rx_rearm_failed();
    }

    /* HAL_UART_IRQHandler는 오류가 난 바이트도 먼저 UART_Receive_IT()로
       완료시킨 뒤 ErrorCallback을 부른다. 이 1-byte 완료 callback에서 다시
       HAL_UART_Receive_IT()를 호출하면 ErrorCode가 NONE으로 초기화되어 바깥
       IRQ handler가 FE/NE/ORE 원인을 잃는다. 재무장 전 값을 복원해 HAL의
       오류 분기와 뒤따르는 ErrorCallback까지 전달한다. */
    huart->ErrorCode |= pending_error;
  }
}

/**
  * @brief  USART2 수신 오류.
  * @note   이 HAL 버전의 동작을 그대로 전제한다.
  *         - FE/NE/PE만 발생한 경우: HAL이 이미 해당 바이트를 UART_Receive_IT로
  *           읽어 갔고 수신은 계속 무장 상태다. ErrorCode도 HAL이 지운다.
  *           여기서 플래그를 다시 지우면 __HAL_UART_CLEAR_*FLAG가 SR 다음 DR을
  *           읽으므로 **막 도착한 정상 바이트를 조용히 삼킨다.** 건드리지 않는다.
  *         - ORE가 섞인 경우: HAL이 수신을 끊는다(blocking error). 흐름제어가 없는
  *           링크라 실제로 일어나므로 여기서 되살리지 않으면 명령이 영영 들어오지
  *           않는다. ORE가 아직 서 있을 때만 SR/DR을 읽어 지운다.
  *         어느 쪽이든 현재 조립 중인 줄은 손상으로 표시해 실행되지 않게 한다.
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uint32_t error_code = huart->ErrorCode;

    app_on_uart_error(error_code);

    if ((error_code & HAL_UART_ERROR_ORE) != 0U)
    {
      if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET)
      {
        __HAL_UART_CLEAR_OREFLAG(huart);
      }
      huart->ErrorCode = HAL_UART_ERROR_NONE;

      if (HAL_UART_Receive_IT(huart, &s_rx_byte, 1U) != HAL_OK)
      {
        app_on_rx_rearm_failed();
      }
    }
  }
}
