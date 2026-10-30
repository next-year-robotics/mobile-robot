/**
  ******************************************************************************
  * @file    imu_app.c
  * @brief   ImuTask. DRDY 대기 → 버스트 읽기 → bias → 게시.
  ******************************************************************************
  */
#include "imu_app.h"

#include "app_config.h"
#include "app_link.h"
#include "control_types.h"
#include "icm42688.h"
#include "imu_bias.h"
#include "imu_types.h"
#include "platform_imu.h"
#include "platform_stm32.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stddef.h>

#define RAD_TO_MDPS  57295.7795f   /* 180/π × 1000 */
#define MPS2_TO_MG   101.971621f   /* 1000 / 9.80665 */

static TaskHandle_t s_task;

/* ISR이 쓰고 ImuTask가 읽는다. 셋은 같은 에지의 짝이므로 critical section 안에서
   한꺼번에 읽는다. */
static volatile uint32_t s_drdy_cycles;
static volatile uint32_t s_drdy_ms;
static volatile uint32_t s_drdy_count;

static imu_app_metrics_t s_metrics;
static imu_bias_t s_bias;

static bool     s_ready;
static uint32_t s_seq;
static uint32_t s_discard;
static uint32_t s_read_fail_streak;
static uint32_t s_last_check_ms;
static uint32_t s_last_drdy_count;

/* 바퀴 정지 판정용 직전 누적 틱 */
static bool    s_have_ticks;
static int64_t s_prev_ticks_left;
static int64_t s_prev_ticks_right;

void imu_app_on_drdy_isr(uint32_t cycles, uint32_t now_ms)
{
  BaseType_t woken = pdFALSE;

  s_drdy_cycles = cycles;
  s_drdy_ms = now_ms;
  s_drdy_count++;

  /* 설정 전 INT1은 open drain으로 떠 있어 에지가 들어올 수 있다. task가 아직 없으면
     시각만 남기고 돌아간다. */
  if (s_task == NULL)
  {
    return;
  }
  vTaskNotifyGiveFromISR(s_task, &woken);
  portYIELD_FROM_ISR(woken);
}

void imu_app_get_metrics(imu_app_metrics_t *out)
{
  *out = s_metrics;
}

static int32_t scale_i32(float value, float scale)
{
  float scaled = value * scale;

  /* 계측용이다. 범위를 넘으면 자른다. ±500 °/s = ±5e5 m°/s라 정상 범위에서는 닿지
     않는다. */
  if (scaled > 2.0e9f)
  {
    return INT32_MAX;
  }
  if (scaled < -2.0e9f)
  {
    return INT32_MIN;
  }
  return (int32_t)scaled;
}

/**
  * @brief  이 표본 동안 바퀴가 멈춰 있었는가.
  * @note   duty가 0이고 누적 틱이 직전 표본과 같다. duty만 보면 사람이 미는 경우를
  *         못 거르고, 틱만 보면 오른쪽 엔코더처럼 죽은 채널에 속는다. 둘 다 본다.
  */
static bool wheels_still(void)
{
  control_status_t status;
  bool still;

  if (!app_link_get_status(&status))
  {
    s_have_ticks = false;
    return false;
  }

  still = s_have_ticks &&
          (status.duty_left_pm == 0) && (status.duty_right_pm == 0) &&
          (status.ticks_left == s_prev_ticks_left) &&
          (status.ticks_right == s_prev_ticks_right);

  s_prev_ticks_left = status.ticks_left;
  s_prev_ticks_right = status.ticks_right;
  s_have_ticks = true;
  return still;
}

static void record_diag(icm42688_status_t status, const icm42688_diag_t *diag)
{
  s_metrics.init_status = (uint32_t)status;
  s_metrics.who_am_i = diag->who_am_i;
  s_metrics.fail_reg = diag->fail_reg;
  s_metrics.fail_wrote = diag->fail_wrote;
  s_metrics.fail_read = diag->fail_read;
}

static bool try_init(void)
{
  icm42688_diag_t diag;
  icm42688_status_t status;
  uint32_t state;

  s_metrics.init_attempts++;
  status = icm42688_init(&diag);
  record_diag(status, &diag);
  s_metrics.spi_error_count = platform_imu_bus_error_count();
  if (status != ICM42688_OK)
  {
    return false;
  }

  /* 설정 전에 떠돌던 에지는 버린다. 그 notification으로 깨어나면 DRDY가 아닌 시각을
     표본에 붙이게 된다. */
  (void)ulTaskNotifyTake(pdTRUE, 0U);
  state = platform_critical_enter();
  s_last_drdy_count = s_drdy_count;
  platform_critical_exit(state);

  s_discard = IMU_STARTUP_DISCARD;
  s_read_fail_streak = 0U;
  s_last_check_ms = platform_now_ms();
  s_have_ticks = false;
  s_ready = true;
  s_metrics.ready = 1U;
  return true;
}

/**
  * @brief  IMU를 잃었다. 다음 바퀴에서 재초기화한다.
  * @note   bias는 지우지 않는다. 같은 칩이 돌아오면 그대로 맞고, 바뀌었으면 다음 정지
  *         창에서 갈린다. 지우면 잠깐의 SPI 끊김에도 /imu/data_raw가 2초씩 멈춘다.
  */
static void lose_imu(void)
{
  s_ready = false;
  s_metrics.ready = 0U;
  s_metrics.reinit_count++;
}

static void publish(const icm42688_si_t *si, uint32_t cycles, uint32_t ms,
                    bool drdy_stamped)
{
  imu_sample_t sample;
  unsigned i;

  (void)imu_bias_update(&s_bias, si->gyro_radps, wheels_still());

  s_seq++;
  sample.seq = s_seq;
  sample.sample_cycles = cycles;
  sample.sample_ms = ms;
  sample.group_delay_us = ICM42688_UI_FILTER_GROUP_DELAY_US;
  imu_bias_apply(&s_bias, si->gyro_radps, sample.gyro_radps);
  for (i = 0U; i < 3U; i++)
  {
    sample.accel_mps2[i] = si->accel_mps2[i];
  }
  sample.temp_c = si->temp_c;
  sample.bias_valid = s_bias.valid;
  sample.drdy_stamped = drdy_stamped;

  (void)app_link_post_imu(&sample);

  s_metrics.sample_count++;
  s_metrics.bias_valid = s_bias.valid ? 1U : 0U;
  s_metrics.bias_update_count = s_bias.update_count;
  s_metrics.bias_reject_count = s_bias.spread_reject_count;
  for (i = 0U; i < 3U; i++)
  {
    s_metrics.bias_mdps[i] = scale_i32(s_bias.bias[i], RAD_TO_MDPS);
    s_metrics.gyro_mdps[i] = scale_i32(sample.gyro_radps[i], RAD_TO_MDPS);
    s_metrics.accel_mg[i] = scale_i32(sample.accel_mps2[i], MPS2_TO_MG);
  }
  s_metrics.temp_cdeg = scale_i32(sample.temp_c, 100.0f);
}

/**
  * @brief  표본 하나. DRDY를 기다리고, 오지 않으면 직접 읽는다.
  */
static void step(void)
{
  icm42688_raw_t raw;
  icm42688_si_t si;
  uint32_t cycles;
  uint32_t ms;
  uint32_t count;
  uint32_t state;
  uint32_t now_ms;
  bool drdy;

  drdy = (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(IMU_DRDY_TIMEOUT_MS)) != 0U);

  state = platform_critical_enter();
  cycles = s_drdy_cycles;
  ms = s_drdy_ms;
  count = s_drdy_count;
  platform_critical_exit(state);

  s_metrics.drdy_count = count;
  if (drdy)
  {
    /* 한 번 읽는 사이 에지가 둘 이상이면 앞의 표본은 이미 덮였다. 데이터와 시각은 둘 다
       마지막 에지의 것이라 짝은 맞다. 잃은 개수만 센다. */
    if ((uint32_t)(count - s_last_drdy_count) > 1U)
    {
      s_metrics.drdy_missed_count += (uint32_t)(count - s_last_drdy_count) - 1U;
    }
  }
  else
  {
    s_metrics.drdy_timeout_count++;
    cycles = platform_dwt_cycles();
    ms = platform_now_ms();
  }
  s_last_drdy_count = count;

  if (!icm42688_read_raw(&raw))
  {
    s_metrics.read_fail_count++;
    s_metrics.spi_error_count = platform_imu_bus_error_count();
    s_read_fail_streak++;
    if (s_read_fail_streak >= IMU_BUS_FAIL_LIMIT)
    {
      lose_imu();
    }
    return;
  }
  s_read_fail_streak = 0U;

  if ((raw.int_status & ICM42688_INT_STATUS_DATA_RDY) == 0U)
  {
    s_metrics.stale_read_count++;
  }

  now_ms = platform_now_ms();
  if ((uint32_t)(now_ms - s_last_check_ms) >= IMU_CHECK_PERIOD_MS)
  {
    icm42688_diag_t diag;
    icm42688_status_t status;

    s_last_check_ms = now_ms;
    status = icm42688_check(&diag);
    if (status != ICM42688_OK)
    {
      /* 이번 표본도 버린다. IMU가 리셋된 뒤라면 레지스터는 마지막 값을 쥔 채
         멈춰 있다. */
      record_diag(status, &diag);
      lose_imu();
      return;
    }
  }

  if (!icm42688_raw_valid(&raw))
  {
    s_metrics.invalid_count++;
    return;
  }
  if (s_discard > 0U)
  {
    s_discard--;
    return;
  }

  icm42688_to_si(&raw, &si);
  publish(&si, cycles, ms, drdy);
}

void imu_task(void *argument)
{
  (void)argument;

  s_task = xTaskGetCurrentTaskHandle();
  imu_bias_init(&s_bias);
  s_ready = false;
  s_seq = 0U;

  for (;;)
  {
    if (!s_ready && !try_init())
    {
      /* 모듈이 빠져 있거나 배선이 틀렸다. 버스를 계속 두드리지 않는다. */
      vTaskDelay(pdMS_TO_TICKS(IMU_INIT_RETRY_MS));
      continue;
    }
    step();
  }
}
