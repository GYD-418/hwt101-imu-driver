/*
 *  ======== hwt101.h ========
 *  HWT101 单轴陀螺仪驱动头文件
 *  通信接口: UART2 (PB15-TX / PB16-RX), 9600bps, 8N1
 */

#ifndef HWT101_H_
#define HWT101_H_

#include <stdint.h>
#include <stdbool.h>

/* ========== 寄存器地址 ========== */
#define HWT101_REG_SAVE         0x00    /* 保存/重启/恢复出厂 */
#define HWT101_REG_CALSE        0x01    /* 手动获取零偏 */
#define HWT101_REG_RRATE        0x03    /* 输出速率 */
#define HWT101_REG_BAUD         0x04    /* 串口波特率 */
#define HWT101_REG_IICADDR      0x1A    /* I2C设备地址 */
#define HWT101_REG_LEDOFF       0x1B    /* 关闭LED灯 */
#define HWT101_REG_READADDR     0x27    /* 读取寄存器 */
#define HWT101_REG_VERSION      0x2E    /* 版本号 */
#define HWT101_REG_YYMM         0x30    /* 年月 */
#define HWT101_REG_DDHH         0x31    /* 日时 */
#define HWT101_REG_MMSS         0x32    /* 分秒 */
#define HWT101_REG_MS           0x33    /* 毫秒 */
#define HWT101_REG_GY           0x38    /* 角速度Y */
#define HWT101_REG_GZ           0x39    /* 角速度Z */
#define HWT101_REG_YAW          0x3F    /* 偏航角 */
#define HWT101_REG_WORKMODE     0x48    /* 工作模式 */
#define HWT101_REG_GYROPTP      0x4A    /* Z轴静止峰峰值 */
#define HWT101_REG_GPTPTIME     0x4B    /* Z轴峰峰值获取时间 */
#define HWT101_REG_GYROBAIS     0x4C    /* Z轴零偏值 */
#define HWT101_REG_GBAISTIME    0x4D    /* Z轴零偏获取时间 */
#define HWT101_REG_GSTATICTHRE  0x4E    /* Z轴静止阈值 */
#define HWT101_REG_GSTATICTIME  0x4F    /* Z轴稳定时间 */
#define HWT101_REG_PGSCALE      0x50    /* Z轴标定因素P */
#define HWT101_REG_GSCALERANGE  0x52    /* Z轴校准角度 */
#define HWT101_REG_GYROCALITHR  0x61    /* 陀螺仪静止阈值 */
#define HWT101_REG_GYROCALTIME  0x63    /* 陀螺仪自动校准时间 */
#define HWT101_REG_KEY          0x69    /* 解锁寄存器 */
#define HWT101_REG_WERROR       0x6A    /* 陀螺仪变化值 */
#define HWT101_REG_WZTIME       0x6E    /* 角速度连续静止时间 */
#define HWT101_REG_WZSTATIC     0x6F    /* 角速度积分阈值 */
#define HWT101_REG_CALIYAW      0x76    /* Z轴角度归零 */

/* ========== 输出速率 (RRATE 寄存器) ========== */
#define HWT101_RRATE_02HZ       0x01
#define HWT101_RRATE_05HZ       0x02
#define HWT101_RRATE_1HZ        0x03
#define HWT101_RRATE_2HZ        0x04
#define HWT101_RRATE_5HZ        0x05
#define HWT101_RRATE_10HZ       0x06    /* 默认 */
#define HWT101_RRATE_20HZ       0x07
#define HWT101_RRATE_50HZ       0x08
#define HWT101_RRATE_100HZ      0x09
#define HWT101_RRATE_200HZ      0x0B
#define HWT101_RRATE_500HZ      0x0C
#define HWT101_RRATE_1000HZ     0x0D

/* ========== 波特率 (BAUD 寄存器) ========== */
#define HWT101_BAUD_4800        0x01
#define HWT101_BAUD_9600        0x02    /* 默认 */
#define HWT101_BAUD_19200       0x03
#define HWT101_BAUD_38400       0x04
#define HWT101_BAUD_57600       0x05
#define HWT101_BAUD_115200      0x06
#define HWT101_BAUD_230400      0x07

/* ========== 工作模式 (WORKMODE 寄存器) ========== */
#define HWT101_MODE_NORMAL      0x00    /* 正常数据模式 */
#define HWT101_MODE_PEAK        0x01    /* 求峰峰值模式 */
#define HWT101_MODE_ZEROBIAS    0x02    /* 求零偏模式 */
#define HWT101_MODE_SCALE       0x03    /* 求标度因素模式 */

/* ========== SAVE 寄存器 ========== */
#define HWT101_SAVE             0x0000  /* 保存设置 */
#define HWT101_REBOOT           0x00FF  /* 重启 */
#define HWT101_FACTORY_RESET    0x0001  /* 恢复出厂 */

/* ========== 数据帧类型 ========== */
#define HWT101_FRAME_HEADER     0x55    /* 读数据帧头 */
#define HWT101_TYPE_GYRO        0x52    /* 角速度数据 */
#define HWT101_TYPE_ANGLE       0x53    /* 角度数据 */
#define HWT101_CMD_HEADER_0     0xFF    /* 写命令帧头0 */
#define HWT101_CMD_HEADER_1     0xAA    /* 写命令帧头1 */

/* ========== 解锁密码 ========== */
#define HWT101_UNLOCK_KEY        0xB588

/* ========== 数据结构（按官方手册） ========== */
typedef struct {
    int16_t yaw;            /* 偏航角 Yaw = ((H<<8)|L) / 32768 * 180 (°) */
    int16_t gyroZ;          /* Z轴角速度  Wz = ((H<<8)|L) / 32768 * 2000 (°/s) */
    bool    newYaw;         /* 本次 update 包含新角度帧 */
    bool    newGyro;        /* 本次 update 包含新角速度帧 */
} HWT101_Data;

/* ========== 帧解析状态机 ========== */
typedef enum {
    HWT101_STATE_WAIT_HEADER = 0,
    HWT101_STATE_WAIT_TYPE,
    HWT101_STATE_WAIT_DATA
} HWT101_ParseState;

/* ========== API 函数声明 ========== */

/**
 * @brief  初始化 UART2 并配置 HWT101
 * @param  baudRate  波特率 (如 9600)
 */
void HWT101_init(uint32_t baudRate);

/**
 * @brief  解锁寄存器 (写任何配置前必须先调用)
 */
void HWT101_unlock(void);

/**
 * @brief  保存设置到 Flash
 */
void HWT101_save(void);

/**
 * @brief  重启传感器
 */
void HWT101_reboot(void);

/**
 * @brief  恢复出厂设置
 */
void HWT101_factoryReset(void);

/**
 * @brief  设置输出速率
 * @param  rate  HWT101_RRATE_xxx 宏定义值
 */
void HWT101_setOutputRate(uint8_t rate);

/**
 * @brief  设置串口波特率
 * @param  baud  HWT101_BAUD_xxx 宏定义值
 */
void HWT101_setBaudRate(uint8_t baud);

/**
 * @brief  设置工作模式
 * @param  mode  HWT101_MODE_xxx 宏定义值
 */
void HWT101_setWorkMode(uint8_t mode);

/**
 * @brief  Z轴角度归零
 */
void HWT101_resetYaw(void);

/**
 * @brief  设置Z轴角度为指定值
 * @param  angle_deg  角度值 (度)
 */
void HWT101_setYaw(float angle_deg);

/**
 * @brief  进入手动零偏校准 (AGV静止时调用)
 */
void HWT101_enterManualCal(void);

/**
 * @brief  退出手动零偏校准 (AGV运动前调用)
 */
void HWT101_exitManualCal(void);

/**
 * @brief  开启自动零偏校准
 */
void HWT101_autoCalibrate(void);

/**
 * @brief  发送读取寄存器命令
 * @param  regAddr  寄存器地址
 */
void HWT101_readRegister(uint8_t regAddr);

/**
 * @brief  获取最新传感器数据 (线程安全)
 * @param  data  输出数据结构指针
 * @return true=数据已更新, false=无新数据
 */
bool HWT101_getData(HWT101_Data *data);

/**
 * @brief  UART2 接收处理 (在主循环或任务中调用)
 *         解析接收到的字节并更新数据
 */
void HWT101_receiveHandler(void);

/**
 * @brief  启动持续接收 (开启UART接收中断或轮询)
 */
void HWT101_startReceive(void);

/**
 * @brief  获取接收到的原始字节数（用于调试）
 */
uint32_t HWT101_getRxByteCount(void);

/**
 * @brief  获取成功解析的帧数（用于调试）
 */
uint32_t HWT101_getFrameCount(void);

/**
 * @brief  获取检测到的帧头 0x55 次数（用于调试）
 */
uint32_t HWT101_getHeaderCount(void);

/**
 * @brief  获取帧类型匹配次数（用于调试）
 */
uint32_t HWT101_getTypeMatchCount(void);

/**
 * @brief  获取各帧类型计数（用于调试）
 *         outA=0x51加速度, outG=0x52角速度, outY=0x53角度
 */
void HWT101_getFrameTypeCounts(uint32_t *outA, uint32_t *outG, uint32_t *outY);

/**
 * @brief  停止持续接收
 */
void HWT101_stopReceive(void);

#endif /* HWT101_H_ */
