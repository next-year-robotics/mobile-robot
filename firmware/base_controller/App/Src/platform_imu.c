/**
  ******************************************************************************
  * @file    platform_imu.c
  * @brief   SPI2 + CS(PB12) + INT1(PB0/EXTI0). HAL 호출은 이 파일 안에만 있다.
  * @note    CubeMX가 만드는 것: hspi2(SPI2, 마스터, 모드 0, 소프트웨어 NSS),
  *          IMU_CS(PB12, 출력, 초기 High), IMU_INT1(PB0, EXTI 상승 에지, EXTI0 NVIC).
  *          라벨 이름이 다르면 여기서 컴파일이 깨진다. 그게 의도다.
  ******************************************************************************
  */
#include "platform_imu.h"

#include "imu_app.h"
#include "platform_stm32.h"

#include "main.h"
#include "spi.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

static uint32_t s_bus_error_count;

static void cs_low(void)
{
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
}

static void cs_high(void)
{
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

/**
  * @brief  CS 한 번에 tx를 내보내고 같은 길이를 받는다.
  * @note   블로킹이다. 18 B는 2.625 MHz에서 55 µs라 DMA를 세우는 비용이 더 크다.
  *         ImuTask가 유일한 호출자라 잠금이 필요 없다.
  */
static bool transfer(const uint8_t *tx, uint8_t *rx, uint16_t length)
{
  HAL_StatusTypeDef status;

  cs_low();
  status = HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)tx, rx, length,
                                   IMU_SPI_TIMEOUT_MS);
  cs_high();

  if (status != HAL_OK)
  {
    s_bus_error_count++;
    return false;
  }
  return true;
}

bool platform_imu_write(uint8_t reg, uint8_t value)
{
  uint8_t tx[2];
  uint8_t rx[2];

  tx[0] = (uint8_t)(reg & 0x7FU);
  tx[1] = value;
  return transfer(tx, rx, 2U);
}

bool platform_imu_read(uint8_t reg, uint8_t *dst, size_t length)
{
  uint8_t tx[1U + PLATFORM_IMU_MAX_READ];
  uint8_t rx[1U + PLATFORM_IMU_MAX_READ];

  if ((length == 0U) || (length > PLATFORM_IMU_MAX_READ))
  {
    return false;
  }

  /* 주소 바이트 뒤로는 무엇을 보내도 된다. 0으로 채워 MOSI를 조용히 둔다. */
  memset(tx, 0, length + 1U);
  tx[0] = (uint8_t)(reg | 0x80U);

  if (!transfer(tx, rx, (uint16_t)(length + 1U)))
  {
    return false;
  }
  memcpy(dst, &rx[1], length);
  return true;
}

void platform_imu_delay_ms(uint32_t ms)
{
  /* tick 1 ms라 1을 주면 0~1 ms 뒤에 깨어날 수 있다. 하나를 더해 최소 ms를 보장한다. */
  vTaskDelay(pdMS_TO_TICKS(ms) + 1U);
}

uint32_t platform_imu_bus_error_count(void)
{
  return s_bus_error_count;
}

/**
  * @brief  EXTI 콜백. HAL에는 하나뿐이라 핀으로 가른다.
  * @note   ISR이다. 시각만 찍고 ImuTask를 깨운다. SPI는 여기서 부르지 않는다.
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == IMU_INT1_Pin)
  {
    imu_app_on_drdy_isr(platform_dwt_cycles(), platform_now_ms());
  }
}
