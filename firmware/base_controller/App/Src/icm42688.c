#include "icm42688.h"

#include "platform_imu.h"

#include <stddef.h>

#define DEG_TO_RAD   0.017453292519943295f
#define STANDARD_G   9.80665f

typedef struct
{
  uint8_t reg;
  uint8_t value;
} reg_write_t;

/* 센서를 켜기 전에 쓰는 설정. 이 순서대로 쓰고 같은 순서로 되읽는다. 센서가 꺼진 상태에서
   ODR·필터를 정해 두어야 켜는 순간부터 그 설정으로 표본이 나온다. */
static const reg_write_t k_config[] = {
  { ICM42688_REG_INTF_CONFIG0,       ICM42688_INTF_CONFIG0_VALUE },
  { ICM42688_REG_INT_CONFIG,         ICM42688_INT_CONFIG_VALUE },
  { ICM42688_REG_INT_CONFIG1,        ICM42688_INT_CONFIG1_VALUE },
  { ICM42688_REG_INT_SOURCE0,        ICM42688_INT_SOURCE0_VALUE },
  { ICM42688_REG_GYRO_CONFIG0,       ICM42688_GYRO_CONFIG0_VALUE },
  { ICM42688_REG_ACCEL_CONFIG0,      ICM42688_ACCEL_CONFIG0_VALUE },
  { ICM42688_REG_GYRO_CONFIG1,       ICM42688_GYRO_CONFIG1_VALUE },
  { ICM42688_REG_ACCEL_CONFIG1,      ICM42688_ACCEL_CONFIG1_VALUE },
  { ICM42688_REG_GYRO_ACCEL_CONFIG0, ICM42688_GYRO_ACCEL_CONFIG0_VALUE },
};

#define CONFIG_COUNT  (sizeof(k_config) / sizeof(k_config[0]))

static void diag_fail(icm42688_diag_t *diag, uint8_t reg, uint8_t wrote,
                      uint8_t read)
{
  diag->fail_reg = reg;
  diag->fail_wrote = wrote;
  diag->fail_read = read;
}

static icm42688_status_t read_who_am_i(icm42688_diag_t *diag)
{
  uint8_t who = 0U;

  if (!platform_imu_read(ICM42688_REG_WHO_AM_I, &who, 1U))
  {
    return ICM42688_ERR_BUS;
  }
  diag->who_am_i = who;
  return (who == ICM42688_WHO_AM_I_VALUE) ? ICM42688_OK : ICM42688_ERR_WHO_AM_I;
}

icm42688_status_t icm42688_init(icm42688_diag_t *diag)
{
  icm42688_status_t status;
  uint8_t value;
  size_t i;

  diag->who_am_i = 0U;
  diag_fail(diag, 0U, 0U, 0U);

  /* MCU만 리셋됐다면 IMU는 이전 상태 그대로다. bank가 0이 아닐 수도 있다고 보고
     먼저 되돌린다. REG_BANK_SEL은 모든 bank에서 같은 주소다(14.59). */
  if (!platform_imu_write(ICM42688_REG_BANK_SEL, 0x00U))
  {
    return ICM42688_ERR_BUS;
  }
  if (!platform_imu_write(ICM42688_REG_DEVICE_CONFIG, ICM42688_SOFT_RESET))
  {
    return ICM42688_ERR_BUS;
  }
  platform_imu_delay_ms(ICM42688_SOFT_RESET_WAIT_MS);

  status = read_who_am_i(diag);
  if (status != ICM42688_OK)
  {
    return status;
  }

  for (i = 0U; i < CONFIG_COUNT; i++)
  {
    if (!platform_imu_write(k_config[i].reg, k_config[i].value))
    {
      return ICM42688_ERR_BUS;
    }
  }

  /* 되읽는다. WHO_AM_I만 맞고 쓰기가 안 먹는 고장(MOSI 단선, SCK 링잉)은 여기서만
     드러난다. 그대로 켜면 기본값(±2000 °/s, 1 kHz)으로 돌면서 그럴듯한 숫자를 낸다. */
  for (i = 0U; i < CONFIG_COUNT; i++)
  {
    if (!platform_imu_read(k_config[i].reg, &value, 1U))
    {
      return ICM42688_ERR_BUS;
    }
    if (value != k_config[i].value)
    {
      diag_fail(diag, k_config[i].reg, k_config[i].value, value);
      return ICM42688_ERR_VERIFY;
    }
  }

  if (!platform_imu_write(ICM42688_REG_PWR_MGMT0, ICM42688_PWR_MGMT0_ON))
  {
    return ICM42688_ERR_BUS;
  }
  /* 꺼짐 → LN 전환 뒤 200 µs 동안 쓰기 금지(14.36). tick 하나로 덮는다. */
  platform_imu_delay_ms(ICM42688_PWR_ON_WAIT_MS);

  if (!platform_imu_read(ICM42688_REG_PWR_MGMT0, &value, 1U))
  {
    return ICM42688_ERR_BUS;
  }
  if (value != ICM42688_PWR_MGMT0_ON)
  {
    diag_fail(diag, ICM42688_REG_PWR_MGMT0, ICM42688_PWR_MGMT0_ON, value);
    return ICM42688_ERR_VERIFY;
  }

  return ICM42688_OK;
}

icm42688_status_t icm42688_check(icm42688_diag_t *diag)
{
  icm42688_status_t status;
  uint8_t value;

  status = read_who_am_i(diag);
  if (status != ICM42688_OK)
  {
    return status;
  }

  /* IMU 혼자 브라운아웃되면 PWR_MGMT0이 0x00(센서 꺼짐)으로 돌아간다. 데이터 레지스터는
     마지막 표본을 쥐고 있어서 값만 봐서는 모른다. */
  if (!platform_imu_read(ICM42688_REG_PWR_MGMT0, &value, 1U))
  {
    return ICM42688_ERR_BUS;
  }
  if (value != ICM42688_PWR_MGMT0_ON)
  {
    diag_fail(diag, ICM42688_REG_PWR_MGMT0, ICM42688_PWR_MGMT0_ON, value);
    return ICM42688_ERR_VERIFY;
  }
  return ICM42688_OK;
}

static int16_t be16(const uint8_t *p)
{
  return (int16_t)(uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

void icm42688_decode(const uint8_t burst[ICM42688_BURST_LEN], icm42688_raw_t *raw)
{
  unsigned i;

  raw->temp = be16(&burst[0]);
  for (i = 0U; i < 3U; i++)
  {
    raw->accel[i] = be16(&burst[2U + (2U * i)]);
    raw->gyro[i] = be16(&burst[8U + (2U * i)]);
  }
  raw->int_status = burst[16];
}

bool icm42688_read_raw(icm42688_raw_t *raw)
{
  uint8_t burst[ICM42688_BURST_LEN];

  if (!platform_imu_read(ICM42688_REG_TEMP_DATA1, burst, sizeof(burst)))
  {
    return false;
  }
  icm42688_decode(burst, raw);
  return true;
}

bool icm42688_raw_valid(const icm42688_raw_t *raw)
{
  unsigned i;

  if (raw->temp == ICM42688_INVALID_SAMPLE)
  {
    return false;
  }
  for (i = 0U; i < 3U; i++)
  {
    if ((raw->accel[i] == ICM42688_INVALID_SAMPLE) ||
        (raw->gyro[i] == ICM42688_INVALID_SAMPLE))
    {
      return false;
    }
  }
  return true;
}

void icm42688_to_si(const icm42688_raw_t *raw, icm42688_si_t *out)
{
  unsigned i;

  for (i = 0U; i < 3U; i++)
  {
    out->gyro_radps[i] =
        ((float)raw->gyro[i] / ICM42688_GYRO_LSB_PER_DPS) * DEG_TO_RAD;
    out->accel_mps2[i] =
        ((float)raw->accel[i] / ICM42688_ACCEL_LSB_PER_G) * STANDARD_G;
  }
  out->temp_c = ((float)raw->temp / ICM42688_TEMP_LSB_PER_C) + ICM42688_TEMP_OFFSET_C;
}
