/**
  ******************************************************************************
  * @file    icm42688.h
  * @brief   ICM-42688-P 레지스터 드라이버. 버스는 platform_imu.h를 거친다.
  * @note    HAL/RTOS 비의존이며 host에서 fake 버스로 시험한다.
  *
  *          레지스터 값의 출처는 TDK DS-000347 Rev 1.6이다. 절 번호를 값 옆에 적어
  *          둔다. 추측으로 채운 값은 없다.
  *
  *          설정 요약
  *            자이로  ±500 °/s (65.5 LSB/(°/s)), ODR 100 Hz, LN 모드
  *            가속도  ±4 g (8192 LSB/g),          ODR 100 Hz, LN 모드
  *            UI 필터 2차(기본값), BW 인덱스 5 → 3 dB 23.8 Hz, 군지연 12.9 ms
  *            INT1    data ready, 펄스 100 µs, push-pull, active high
  *            I2C     끈다 (SPI만 쓴다)
  *
  *          ±500 °/s인 이유: 운용 상한 6000 tps로 제자리 회전하면 약 280 °/s다
  *          (r 0.050 m, 유효 L 0.30912 m). ±250이면 포화한다.
  ******************************************************************************
  */
#ifndef ICM42688_H
#define ICM42688_H

#include <stdbool.h>
#include <stdint.h>

/* ---- bank 0 레지스터 (DS 13.1절) ---- */
#define ICM42688_REG_DEVICE_CONFIG       0x11U
#define ICM42688_REG_INT_CONFIG          0x14U
#define ICM42688_REG_TEMP_DATA1          0x1DU
#define ICM42688_REG_INT_STATUS          0x2DU
#define ICM42688_REG_INTF_CONFIG0        0x4CU
#define ICM42688_REG_PWR_MGMT0           0x4EU
#define ICM42688_REG_GYRO_CONFIG0        0x4FU
#define ICM42688_REG_ACCEL_CONFIG0       0x50U
#define ICM42688_REG_GYRO_CONFIG1        0x51U
#define ICM42688_REG_GYRO_ACCEL_CONFIG0  0x52U
#define ICM42688_REG_ACCEL_CONFIG1       0x53U
#define ICM42688_REG_INT_CONFIG1         0x64U
#define ICM42688_REG_INT_SOURCE0         0x65U
#define ICM42688_REG_WHO_AM_I            0x75U
#define ICM42688_REG_BANK_SEL            0x76U

#define ICM42688_SPI_READ                0x80U
#define ICM42688_WHO_AM_I_VALUE          0x47U   /* 14.58 */

/* ---- 쓰는 값 ---- */

/* 14.1 SOFT_RESET_CONFIG. 쓴 뒤 1 ms 동안 레지스터를 건드리지 않는다. */
#define ICM42688_SOFT_RESET              0x01U
#define ICM42688_SOFT_RESET_WAIT_MS      2U

/* 14.34 기본 0x30(데이터·FIFO count 빅엔디언)을 두고 UI_SIFS_CFG = 11(I2C 끔). */
#define ICM42688_INTF_CONFIG0_VALUE      0x33U

/* 14.3 INT1: bit2 펄스(0), bit1 push-pull(1), bit0 active high(1). 래치 모드는
   EXTI 에지 하나를 놓치면 핀이 High에 붙어 다음 에지가 안 온다. 펄스는 읽기와
   무관하게 표본마다 다시 뜬다. */
#define ICM42688_INT_CONFIG_VALUE        0x03U

/* 14.50 INT_ASYNC_RESET(bit4)을 0으로. 데이터시트 12.6절이 INT 핀 동작에 필수라고
   적었다. 펄스 100 µs·de-assert 100 µs는 ODR < 4 kHz 기본값 그대로다. */
#define ICM42688_INT_CONFIG1_VALUE       0x00U

/* 14.51 UI_DRDY_INT1_EN(bit3)만. 기본값 0x10(RESET_DONE)은 뺀다. */
#define ICM42688_INT_SOURCE0_VALUE       0x08U

/* 14.37 / 14.38 FS_SEL은 bit7:5, ODR은 bit3:0. 1000b = 100 Hz. */
#define ICM42688_GYRO_FS_500DPS          0x02U
#define ICM42688_ACCEL_FS_4G             0x02U
#define ICM42688_ODR_100HZ               0x08U
#define ICM42688_GYRO_CONFIG0_VALUE   \
  ((uint8_t)((ICM42688_GYRO_FS_500DPS << 5) | ICM42688_ODR_100HZ))
#define ICM42688_ACCEL_CONFIG0_VALUE  \
  ((uint8_t)((ICM42688_ACCEL_FS_4G << 5) | ICM42688_ODR_100HZ))

/* 14.40 BW = max(400 Hz, ODR)/16. 2차 필터 기준 3 dB 23.8 Hz, NBW 25.9 Hz(5.3절 표).
   ODR 100 Hz의 나이퀴스트 50 Hz 아래로 충분히 내려 모터 진동이 접히지 않게 한다.
   요레이트에 필요한 대역(수 Hz)은 그대로 지난다. */
#define ICM42688_UI_FILT_BW_ODR16        0x05U
#define ICM42688_GYRO_ACCEL_CONFIG0_VALUE \
  ((uint8_t)((ICM42688_UI_FILT_BW_ODR16 << 4) | ICM42688_UI_FILT_BW_ODR16))

/* 14.39 / 14.41 UI 필터 차수. 둘 다 리셋값(2차)이지만 군지연 보정이 이 차수에 묶여
   있으므로 명시적으로 쓰고 되읽는다.
     GYRO_CONFIG1  0x16: TEMP_FILT_BW 000, GYRO_UI_FILT_ORD 01(2차), DEC2_M2 10
     ACCEL_CONFIG1 0x0D: ACCEL_UI_FILT_ORD 01(2차), DEC2_M2 10, bit0 예약(리셋값 1) */
#define ICM42688_GYRO_CONFIG1_VALUE      0x16U
#define ICM42688_ACCEL_CONFIG1_VALUE     0x0DU

/* 위 필터의 DC 군지연. 5.3절 "Group Delay @DC, 2nd order" 표의 ODR 100 Hz, BW 인덱스 5
   칸이다. 자이로와 가속이 같은 표를 쓴다. DRDY는 필터 출력이 준비된 시각이고, 그 값이
   나타내는 물리량은 이만큼 앞선 것이다. stamp에서 뺀다. ODR·BW·차수 중 하나라도
   바꾸면 표에서 다시 읽어 고친다. */
#define ICM42688_UI_FILTER_GROUP_DELAY_US 12900U

/* 14.36 GYRO_MODE = 11(LN), ACCEL_MODE = 11(LN), 온도 켬. 켠 뒤 200 µs 동안 쓰기 금지. */
#define ICM42688_PWR_MGMT0_ON            0x0FU
#define ICM42688_PWR_ON_WAIT_MS          1U

/* 감도 (DS 표 1, 표 2, 3.3절) */
#define ICM42688_GYRO_LSB_PER_DPS        65.5f
#define ICM42688_ACCEL_LSB_PER_G         8192.0f
#define ICM42688_TEMP_LSB_PER_C          132.48f
#define ICM42688_TEMP_OFFSET_C           25.0f

/* 표본 버스트: TEMP_DATA1(0x1D) ~ INT_STATUS(0x2D) = 17 B.
     [0..1] 온도  [2..7] 가속 XYZ  [8..13] 자이로 XYZ  [14..15] TMST_FSYNC  [16] INT_STATUS
   INT_STATUS까지 한 번에 읽는 이유: DATA_RDY_INT를 보면 같은 표본을 두 번 읽었는지
   알 수 있다. 펄스 모드에서는 이 비트를 지우지 않아도 핀은 다시 뜨지만, 읽어 두면
   fallback 경로의 중복 표본을 셀 수 있다. */
#define ICM42688_BURST_LEN               17U
#define ICM42688_INT_STATUS_DATA_RDY     0x08U

/* 14.34 Invalid Data Generation: 전원 투입 뒤 첫 ODR 표본 전에는 리셋값 0x8000이
   읽힌다. 유효 표본의 -32768은 -32766으로 바뀌어 나오므로 이 값은 "아직 없음"이다. */
#define ICM42688_INVALID_SAMPLE          (-32768)

typedef enum
{
  ICM42688_OK = 0,
  ICM42688_ERR_BUS,       /* SPI 트랜잭션 실패 */
  ICM42688_ERR_WHO_AM_I,  /* 응답은 왔는데 0x47이 아니다 (MISO 단선이면 0x00/0xFF) */
  ICM42688_ERR_VERIFY     /* 쓴 값과 되읽은 값이 다르다 (MOSI/SCK 문제) */
} icm42688_status_t;

/** @brief 실패 원인을 SWD로 보기 위한 값. */
typedef struct
{
  uint8_t who_am_i;       /* 마지막으로 읽은 WHO_AM_I */
  uint8_t fail_reg;       /* VERIFY 실패 레지스터 */
  uint8_t fail_wrote;     /* 그 레지스터에 쓴 값 */
  uint8_t fail_read;      /* 되읽은 값 */
} icm42688_diag_t;

typedef struct
{
  int16_t temp;
  int16_t accel[3];
  int16_t gyro[3];
  uint8_t int_status;
} icm42688_raw_t;

typedef struct
{
  float gyro_radps[3];
  float accel_mps2[3];
  float temp_c;
} icm42688_si_t;

/**
  * @brief  소프트 리셋 → WHO_AM_I → 설정 → 되읽기 확인 → 센서 켜기.
  * @note   platform_imu_delay_ms를 부르므로 task 문맥에서만 부른다. 실패하면 센서는
  *         꺼진 상태로 남는다(리셋 직후 기본값).
  */
icm42688_status_t icm42688_init(icm42688_diag_t *diag);

/**
  * @brief  IMU가 여전히 우리가 설정한 상태인지 본다 (WHO_AM_I, PWR_MGMT0).
  * @retval IMU가 리셋됐으면 ICM42688_ERR_VERIFY. 호출자는 재초기화한다.
  */
icm42688_status_t icm42688_check(icm42688_diag_t *diag);

/** @brief 표본 버스트 하나를 읽어 해석한다. */
bool icm42688_read_raw(icm42688_raw_t *raw);

/** @brief 17 B 버스트를 해석한다. 데이터는 빅엔디언이다(INTF_CONFIG0 기본값). */
void icm42688_decode(const uint8_t burst[ICM42688_BURST_LEN], icm42688_raw_t *raw);

/** @brief 온도·가속·자이로 중 하나라도 "아직 없음" 값이면 false. */
bool icm42688_raw_valid(const icm42688_raw_t *raw);

/** @brief 원시값을 SI로 바꾼다. 축은 센서 축 그대로다. */
void icm42688_to_si(const icm42688_raw_t *raw, icm42688_si_t *out);

#endif /* ICM42688_H */
