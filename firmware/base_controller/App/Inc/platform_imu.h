/**
  ******************************************************************************
  * @file    platform_imu.h
  * @brief   IMU SPI2(PB13/14/15) + CS(PB12) + INT1(PB0) adapter. HAL 타입은 나오지 않는다.
  * @note    ImuTask가 SPI2의 유일한 소유자다. icm42688.c는 이 헤더의 함수만 부르고
  *          host 테스트는 같은 이름의 fake로 대신한다(platform_stm32.h와 같은 규칙).
  *
  *          INT1 상승 에지(EXTI0)는 platform_imu.c의 HAL_GPIO_EXTI_Callback이 받아
  *          imu_app_on_drdy_isr()로 넘긴다.
  ******************************************************************************
  */
#ifndef PLATFORM_IMU_H
#define PLATFORM_IMU_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 한 트랜잭션에서 읽을 수 있는 최대 바이트. 표본 버스트(17 B)를 담는다. */
#define PLATFORM_IMU_MAX_READ  24U

/**
  * @brief  레지스터 하나를 쓴다. CS를 내리고 [reg, value]를 보낸 뒤 올린다.
  * @retval SPI 타임아웃/오류면 false.
  */
bool platform_imu_write(uint8_t reg, uint8_t value);

/**
  * @brief  reg부터 length 바이트를 연속으로 읽는다(자동 증가).
  * @retval length가 PLATFORM_IMU_MAX_READ를 넘거나 SPI가 실패하면 false.
  */
bool platform_imu_read(uint8_t reg, uint8_t *dst, size_t length);

/** @brief 초기화 순서의 대기. task 문맥에서만 부른다. */
void platform_imu_delay_ms(uint32_t ms);

/** @brief SPI 오류 누적. SWD 계측용이다. */
uint32_t platform_imu_bus_error_count(void);

#endif /* PLATFORM_IMU_H */
