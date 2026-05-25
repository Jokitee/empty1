/**
 * @file    lsm6dsv.h
 * @brief   LSM6DSV 六轴 IMU 驱动（加速度计 + 陀螺仪）
 * @note    通过 I2C1 接口与 MSPM0G3507 通信
 *          SDA: PA16, SCL: PA17
 */

#ifndef LSM6DSV_H
#define LSM6DSV_H

#include <stdint.h>
#include <stdbool.h>

/* ========== I2C 地址 ========== */
#define LSM6DSV_I2C_ADDR_SA0_LOW    0x6A    /* SA0 = GND */
#define LSM6DSV_I2C_ADDR_SA0_HIGH   0x6B    /* SA0 = VCC */
#define LSM6DSV_I2C_ADDR            LSM6DSV_I2C_ADDR_SA0_HIGH

/* ========== 静态校准预设配置 ========== */
#define IMU_USE_PRESET_CALIBRATION   1       /* 0 = 开机静置1秒自动校准（推荐首次使用），1 = 使用下方预设偏置 */
#define IMU_PRESET_BIAS_X            0.91f    /* 陀螺仪 X 轴静态漂移预设值 */
#define IMU_PRESET_BIAS_Y            0.298f    /* 陀螺仪 Y 轴静态漂移预设值 */
#define IMU_PRESET_BIAS_Z            0.205f    /* 陀螺仪 Z 轴静态漂移预设值 */
#define IMU_PRESET_BIAS_ROLL         2.15f    /* 加速度计 Roll 零偏预设值 */
#define IMU_PRESET_BIAS_PITCH        0.02f    /* 加速度计 Pitch 零偏预设值 */

/* ========== 寄存器地址 ========== */
#define LSM6DSV_REG_WHO_AM_I        0x0F
#define LSM6DSV_WHO_AM_I_VALUE      0x70

/* ---- 功能使能 / 状态 ---- */
#define LSM6DSV_REG_CTRL1           0x10    /* 加速度计 ODR + FS */
#define LSM6DSV_REG_CTRL2           0x11    /* 陀螺仪 ODR + FS */
#define LSM6DSV_REG_CTRL3           0x12    /* BDU / IF_INC / SW_RESET */
#define LSM6DSV_REG_CTRL4           0x13
#define LSM6DSV_REG_CTRL5           0x14
#define LSM6DSV_REG_CTRL6           0x15
#define LSM6DSV_REG_CTRL7           0x16
#define LSM6DSV_REG_CTRL8           0x17
#define LSM6DSV_REG_CTRL9           0x18
#define LSM6DSV_REG_CTRL10          0x19

#define LSM6DSV_REG_STATUS_REG      0x1E

/* ---- 温度 ---- */
#define LSM6DSV_REG_OUT_TEMP_L      0x20
#define LSM6DSV_REG_OUT_TEMP_H      0x21

/* ---- 陀螺仪输出 ---- */
#define LSM6DSV_REG_OUTX_L_G        0x22
#define LSM6DSV_REG_OUTX_H_G        0x23
#define LSM6DSV_REG_OUTY_L_G        0x24
#define LSM6DSV_REG_OUTY_H_G        0x25
#define LSM6DSV_REG_OUTZ_L_G        0x26
#define LSM6DSV_REG_OUTZ_H_G        0x27

/* ---- 加速度计输出 ---- */
#define LSM6DSV_REG_OUTX_L_XL       0x28
#define LSM6DSV_REG_OUTX_H_XL       0x29
#define LSM6DSV_REG_OUTY_L_XL       0x2A
#define LSM6DSV_REG_OUTY_H_XL       0x2B
#define LSM6DSV_REG_OUTZ_L_XL       0x2C
#define LSM6DSV_REG_OUTZ_H_XL       0x2D

/* ========== 加速度计量程 ========== */
typedef enum {
    LSM6DSV_XL_FS_2G  = 0x00,     /* ±2g  */
    LSM6DSV_XL_FS_4G  = 0x01,     /* ±4g  */
    LSM6DSV_XL_FS_8G  = 0x02,     /* ±8g  */
    LSM6DSV_XL_FS_16G = 0x03      /* ±16g */
} LSM6DSV_XL_FS_t;

/* ========== 陀螺仪量程 (CTRL6[3:0] = FS_G) ========== */
typedef enum {
    LSM6DSV_G_FS_125DPS  = 0x00,  /* ±125 dps (default) */
    LSM6DSV_G_FS_250DPS  = 0x01,  /* ±250 dps  */
    LSM6DSV_G_FS_500DPS  = 0x02,  /* ±500 dps  */
    LSM6DSV_G_FS_1000DPS = 0x03,  /* ±1000 dps */
    LSM6DSV_G_FS_2000DPS = 0x04,  /* ±2000 dps */
    LSM6DSV_G_FS_4000DPS = 0x0C   /* ±4000 dps */
} LSM6DSV_G_FS_t;

/* ========== 输出数据率 (ODR) ========== */
typedef enum {
    LSM6DSV_ODR_OFF      = 0x00,
    LSM6DSV_ODR_1_875HZ  = 0x01,     /* 1.875 Hz */
    LSM6DSV_ODR_7_5HZ    = 0x02,     /* 7.5 Hz   */
    LSM6DSV_ODR_15HZ     = 0x03,     /* 15 Hz    */
    LSM6DSV_ODR_30HZ     = 0x04,     /* 30 Hz    */
    LSM6DSV_ODR_60HZ     = 0x05,     /* 60 Hz    */
    LSM6DSV_ODR_120HZ    = 0x06,     /* 120 Hz   */
    LSM6DSV_ODR_240HZ    = 0x07,     /* 240 Hz   */
    LSM6DSV_ODR_480HZ    = 0x08,     /* 480 Hz   */
    LSM6DSV_ODR_960HZ    = 0x09,     /* 960 Hz   */
    LSM6DSV_ODR_1920HZ   = 0x0A,     /* 1920 Hz  */
    LSM6DSV_ODR_3840HZ   = 0x0B,     /* 3840 Hz  */
    LSM6DSV_ODR_7680HZ   = 0x0C      /* 7680 Hz  */
} LSM6DSV_ODR_t;


/* ========== 3轴数据结构 ========== */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} LSM6DSV_AxesRaw_t;

typedef struct {
    float x;
    float y;
    float z;
} LSM6DSV_Axes_t;

/* ========== 姿态数据结构 ========== */
typedef struct {
    float roll;   // 横滚角 (Roll)，单位：度
    float pitch;  // 俯仰角 (Pitch)，单位：度
    float yaw;    // 偏航角 (Yaw)，单位：度
} LSM6DSV_Attitude_t;

/* ========== 设备句柄 ========== */
typedef struct {
    uint8_t         i2c_addr;
    LSM6DSV_XL_FS_t xl_fs;
    LSM6DSV_G_FS_t  g_fs;
    float           xl_sensitivity;   /* mg/LSB */
    float           g_sensitivity;    /* mdps/LSB */
    float           gyro_bias_x;      /* 陀螺仪 X 轴静态漂移 */
    float           gyro_bias_y;      /* 陀螺仪 Y 轴静态漂移 */
    float           gyro_bias_z;      /* 陀螺仪 Z 轴静态漂移 */
    float           accel_bias_roll;  /* 加速度计 Roll 零偏 */
    float           accel_bias_pitch; /* 加速度计 Pitch 零偏 */
    
    /* 一阶低通滤波器状态变量 */
    float           filtered_gx;      /* 滤波后的 X 轴角速度 */
    float           filtered_gy;      /* 滤波后的 Y 轴角速度 */
    float           filtered_gz;      /* 滤波后的 Z 轴角速度 */

    /* 诊断字段：帮助定位陀螺仪数据问题 */
    uint32_t        diag_read_total;  /* UpdateAttitude 总调用次数 */
    uint32_t        diag_read_fail;   /* ReadAll 返回 false 的次数 */
    float           diag_raw_gx;      /* 最近一次成功读取的原始 gx（减偏置前）*/
    float           diag_raw_gy;      /* 最近一次成功读取的原始 gy */
    float           diag_raw_gz;      /* 最近一次成功读取的原始 gz */
} LSM6DSV_Handle_t;

/* ========== API 函数 ========== */

/**
 * @brief  初始化 LSM6DSV
 * @param  handle 设备句柄
 * @return true=成功, false=WHO_AM_I 校验失败
 */
bool LSM6DSV_Init(LSM6DSV_Handle_t *handle);

/**
 * @brief  软件复位
 */
void LSM6DSV_Reset(LSM6DSV_Handle_t *handle);

/**
 * @brief  读取 WHO_AM_I 寄存器
 */
uint8_t LSM6DSV_ReadWhoAmI(LSM6DSV_Handle_t *handle);

/**
 * @brief  配置加速度计量程和 ODR
 */
bool LSM6DSV_ConfigAccel(LSM6DSV_Handle_t *handle,
                          LSM6DSV_ODR_t odr, LSM6DSV_XL_FS_t fs);

/**
 * @brief  配置陀螺仪量程和 ODR
 */
bool LSM6DSV_ConfigGyro(LSM6DSV_Handle_t *handle,
                          LSM6DSV_ODR_t odr, LSM6DSV_G_FS_t fs);


/**
 * @brief  一次读取全部数据（加速度 + 陀螺仪 + 温度）
 */
bool LSM6DSV_ReadAll(LSM6DSV_Handle_t *handle,
                      LSM6DSV_Axes_t *accel,
                      LSM6DSV_Axes_t *gyro,
                      float *temp);

/* ========== 滤波算法与姿态解算 API ========== */

/**
 * @brief  开机 1 秒车辆姿态校验与陀螺仪校准
 * @param  handle LSM6DSV 句柄
 * @param  attitude 姿态结构体（初始化为校准后的静态 Roll/Pitch，Yaw归零）
 * @return bool 是否通过校验（true = 通过，false = 倾斜过大或在振动/运动中）
 */
bool LSM6DSV_BootCalibrate(LSM6DSV_Handle_t *handle, LSM6DSV_Attitude_t *attitude);

/**
 * @brief  直接载入预设的零偏校准数据
 */
void LSM6DSV_LoadPresetCalibrate(LSM6DSV_Handle_t *handle, LSM6DSV_Attitude_t *attitude);

/**
 * @brief  姿态解算互补滤波更新
 * @param  handle LSM6DSV 句柄
 * @param  attitude 姿态输出结构体
 * @param  dt 两次调用之间的时间间隔（秒）
 */
void LSM6DSV_UpdateAttitude(LSM6DSV_Handle_t *handle, LSM6DSV_Attitude_t *attitude, float dt);

/* ========== 底层 I2C 读写（供内部使用，也可外部调用） ========== */
bool LSM6DSV_ReadReg(LSM6DSV_Handle_t *handle, uint8_t reg, uint8_t *buf, uint8_t len);
bool LSM6DSV_WriteReg(LSM6DSV_Handle_t *handle, uint8_t reg, uint8_t val);

#endif /* LSM6DSV_H */