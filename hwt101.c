/*
 *  ======== hwt101.c ========
 *  HWT101 单轴陀螺仪驱动实现
 *  UART2: PB15(TX) / PB16(RX), 9600bps, 8N1
 */

#include "hwt101.h"
#include "ti_msp_dl_config.h"
#include <string.h>

/* ========== 硬件配置 ========== */
#define HWT101_UART           UART2
#define HWT101_BAUD_DEFAULT   9600
#define HWT101_CLOCK_FREQ     32000000UL  /* 32 MHz BUSCLK */

/* IOMUX 引脚定义 */
#define HWT101_TX_PINCM       IOMUX_PINCM32   /* PB15 = UART2_TX */
#define HWT101_TX_PIN         DL_GPIO_PIN_15
#define HWT101_TX_PORT        GPIOB

#define HWT101_RX_PINCM       IOMUX_PINCM33   /* PB16 = UART2_RX */
#define HWT101_RX_PIN         DL_GPIO_PIN_16
#define HWT101_RX_PORT        GPIOB

/* ========== 接收缓冲区 ========== */
#define HWT101_RX_BUF_SIZE    11  /* 一帧数据 11 字节 */

/* ========== 静态变量 ========== */
static volatile HWT101_Data   g_data;
static volatile bool          g_dataReady = false;

static volatile uint8_t       g_rxBuf[HWT101_RX_BUF_SIZE];
static volatile uint8_t       g_rxCnt = 0;
static volatile HWT101_ParseState g_parseState = HWT101_STATE_WAIT_HEADER;
static volatile bool          g_receiveEnabled = false;

/* ========== 调试变量 ========== */
static volatile uint32_t      g_rxByteCount = 0;
static volatile uint32_t      g_frameCount = 0;
static volatile uint32_t      g_typeCountA = 0;   /* 0x51 加速度 */
static volatile uint32_t      g_typeCountG = 0;   /* 0x52 角速度 */
static volatile uint32_t      g_typeCountY = 0;   /* 0x53 角度 */
static volatile uint32_t      g_headerCount = 0;
static volatile uint32_t      g_typeMatch = 0;

/* ========== 内部函数 ========== */

/**
 * @brief  发送一个字节 (阻塞)
 */
static void HWT101_sendByte(uint8_t data)
{
    DL_UART_transmitDataBlocking(HWT101_UART, data);
}

/**
 * @brief  延时 (毫秒级, 使用 DriverLib delay_cycles)
 */
static void HWT101_delayMs(uint32_t ms)
{
    /* 32 MHz -> 32000 cycles per ms */
    delay_cycles(32000 * ms);
}

/**
 * @brief  发送写命令 (5字节帧)
 *         FF AA ADDR DATAL DATAH
 */
static void HWT101_sendCmd(uint8_t addr, uint16_t data)
{
    uint8_t dataL = (uint8_t)(data & 0xFF);
    uint8_t dataH = (uint8_t)((data >> 8) & 0xFF);

    HWT101_sendByte(HWT101_CMD_HEADER_0);  /* 0xFF */
    HWT101_sendByte(HWT101_CMD_HEADER_1);  /* 0xAA */
    HWT101_sendByte(addr);
    HWT101_sendByte(dataL);
    HWT101_sendByte(dataH);
}

/**
 * @brief  解析一帧接收数据（按官方数据手册）
 *
 *         角度帧 0x53: [0x55] [0x53] [0x00] [0x00] [0x00] [0x00] [YawL] [YawH] [VL] [VH] [SUM]
 *         Yaw 在 buf[6]/buf[7]
 *
 *         角速度帧0x52: [0x55] [0x52] [0x00] [0x00] [RWzL] [RWzH] [WzL] [WzH] [0x00] [0x00] [SUM]
 *         Wz 在 buf[6]/buf[7],  RWz 在 buf[4]/buf[5]
 */
static void HWT101_parseFrame(const uint8_t *buf)
{
    uint8_t type = buf[1];

    switch (type) {
        case 0x52:  /* 角速度帧 */
            g_data.gyroZ  = (int16_t)((uint16_t)buf[7] << 8 | buf[6]);
            g_data.newGyro = true;
            g_typeCountG++;
            break;
        case 0x53:  /* 角度帧 */
            g_data.yaw   = (int16_t)((uint16_t)buf[7] << 8 | buf[6]);
            g_data.newYaw = true;
            g_typeCountY++;
            break;
    }

    g_frameCount++;
}

/* ========== 公共 API 实现 ========== */

void HWT101_init(uint32_t baudRate)
{
    /* 使能 UART2 电源 */
    DL_UART_enablePower(HWT101_UART);
    delay_cycles(POWER_STARTUP_DELAY);

    /* 配置 UART: 8N1, 收发, 无流控 */
    const DL_UART_Config uartConfig = {
        .mode         = DL_UART_MODE_NORMAL,
        .direction    = DL_UART_DIRECTION_TX_RX,
        .flowControl  = DL_UART_FLOW_CONTROL_NONE,
        .parity       = DL_UART_PARITY_NONE,
        .wordLength   = DL_UART_WORD_LENGTH_8_BITS,
        .stopBits     = DL_UART_STOP_BITS_ONE,
    };
    DL_UART_init(HWT101_UART, &uartConfig);

    /* 配置时钟: BUSCLK, 分频比 1:1 */
    const DL_UART_ClockConfig clockConfig = {
        .clockSel    = DL_UART_CLOCK_BUSCLK,
        .divideRatio = DL_UART_CLOCK_DIVIDE_RATIO_1,
    };
    DL_UART_setClockConfig(HWT101_UART, &clockConfig);

    /* 配置波特率 */
    DL_UART_configBaudRate(HWT101_UART, HWT101_CLOCK_FREQ, baudRate);

    /* 使能 FIFO */
    DL_UART_enableFIFOs(HWT101_UART);

    /* 配置 IOMUX 引脚 */
    /* TX: PB15 (PINCM32) -> UART2_TX */
    DL_GPIO_initPeripheralOutputFunction(HWT101_TX_PINCM,
                                         IOMUX_PINCM32_PF_UART2_TX);
    /* RX: PB16 (PINCM33) -> UART2_RX */
    DL_GPIO_initPeripheralInputFunction(HWT101_RX_PINCM,
                                        IOMUX_PINCM33_PF_UART2_RX);

    /* 使能 UART */
    DL_UART_enable(HWT101_UART);

    /* 清空接收 FIFO */
    uint8_t dummy;
    while (!DL_UART_isRXFIFOEmpty(HWT101_UART)) {
        DL_UART_receiveDataCheck(HWT101_UART, &dummy);
    }

    /* 初始化数据 */
    memset((void *)&g_data, 0, sizeof(g_data));
    g_dataReady = false;
    g_rxCnt = 0;
    g_parseState = HWT101_STATE_WAIT_HEADER;
}

void HWT101_unlock(void)
{
    HWT101_sendCmd(HWT101_REG_KEY, HWT101_UNLOCK_KEY);
    HWT101_delayMs(200);
}

void HWT101_save(void)
{
    HWT101_sendCmd(HWT101_REG_SAVE, HWT101_SAVE);
    HWT101_delayMs(200);
}

void HWT101_reboot(void)
{
    HWT101_sendCmd(HWT101_REG_SAVE, HWT101_REBOOT);
    HWT101_delayMs(500);
}

void HWT101_factoryReset(void)
{
    HWT101_sendCmd(HWT101_REG_SAVE, HWT101_FACTORY_RESET);
    HWT101_delayMs(500);
}

void HWT101_setOutputRate(uint8_t rate)
{
    HWT101_unlock();
    HWT101_delayMs(100);
    HWT101_sendCmd(HWT101_REG_RRATE, rate);
    HWT101_delayMs(100);
    HWT101_save();
}

void HWT101_setBaudRate(uint8_t baud)
{
    HWT101_unlock();
    HWT101_delayMs(100);
    HWT101_sendCmd(HWT101_REG_BAUD, baud);
    HWT101_delayMs(100);
    HWT101_save();
}

void HWT101_setWorkMode(uint8_t mode)
{
    HWT101_sendCmd(HWT101_REG_WORKMODE, mode);
    HWT101_delayMs(200);
}

void HWT101_resetYaw(void)
{
    HWT101_unlock();
    HWT101_delayMs(200);
    HWT101_sendCmd(HWT101_REG_CALIYAW, 0x0000);
    HWT101_delayMs(500);
    HWT101_save();
}

void HWT101_setYaw(float angle_deg)
{
    int16_t raw = (int16_t)(angle_deg * 32768.0f / 180.0f);

    HWT101_unlock();
    HWT101_delayMs(200);
    HWT101_sendCmd(HWT101_REG_CALIYAW, (uint16_t)raw);
    HWT101_delayMs(200);
    HWT101_save();
}

void HWT101_enterManualCal(void)
{
    /* AGV 静止后发送, 不需要解锁和保存 */
    HWT101_sendCmd(HWT101_REG_CALSE, 0x0001);
}

void HWT101_exitManualCal(void)
{
    /* AGV 运动前发送, 不需要解锁和保存 */
    HWT101_sendCmd(HWT101_REG_CALSE, 0x0004);
}

void HWT101_autoCalibrate(void)
{
    HWT101_unlock();
    HWT101_delayMs(200);
    HWT101_setWorkMode(HWT101_MODE_ZEROBIAS);
    HWT101_delayMs(500);
    HWT101_save();
    HWT101_delayMs(20000);  /* 等待自动校准完成 (约20秒) */
}

void HWT101_readRegister(uint8_t regAddr)
{
    /* 发送读取命令: FF AA 27 ADDR 00 */
    HWT101_sendCmd(HWT101_REG_READADDR, regAddr);
}

void HWT101_receiveHandler(void)
{
    uint8_t byte;

    if (!g_receiveEnabled) {
        return;
    }

    /* 非阻塞读取所有可用字节 */
    while (!DL_UART_isRXFIFOEmpty(HWT101_UART)) {
        if (!DL_UART_receiveDataCheck(HWT101_UART, &byte)) {
            break;
        }
        g_rxByteCount++;  /* 每收到一个字节就计数 */

        switch (g_parseState) {
        case HWT101_STATE_WAIT_HEADER:
            if (byte == HWT101_FRAME_HEADER) {  /* 0x55 */
                g_rxBuf[0] = byte;
                g_rxCnt = 1;
                g_parseState = HWT101_STATE_WAIT_TYPE;
                g_headerCount++;
            }
            break;

        case HWT101_STATE_WAIT_TYPE:
            if (byte == HWT101_TYPE_GYRO || byte == HWT101_TYPE_ANGLE) {
                g_rxBuf[1] = byte;
                g_rxCnt = 2;
                g_parseState = HWT101_STATE_WAIT_DATA;
                g_typeMatch++;
            } else {
                g_parseState = HWT101_STATE_WAIT_HEADER;
            }
            break;

        case HWT101_STATE_WAIT_DATA:
            g_rxBuf[g_rxCnt] = byte;
            g_rxCnt++;
            if (g_rxCnt >= HWT101_RX_BUF_SIZE) {
                /* 一帧完整, 解析数据 */
                HWT101_parseFrame((const uint8_t *)g_rxBuf);
                g_rxCnt = 0;
                g_parseState = HWT101_STATE_WAIT_HEADER;
            }
            break;

        default:
            g_parseState = HWT101_STATE_WAIT_HEADER;
            break;
        }
    }
}

bool HWT101_getData(HWT101_Data *data)
{
    if (data == NULL) {
        return false;
    }

    /* 关中断保护 (Cortex-M0+) */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (g_data.newYaw || g_data.newGyro) {
        data->yaw    = g_data.yaw;
        data->newYaw = g_data.newYaw;
        data->newGyro = g_data.newGyro;
        data->gyroZ  = g_data.gyroZ;
        g_data.newYaw = false;
        g_data.newGyro = false;
        __set_PRIMASK(primask);
        return true;
    }
    __set_PRIMASK(primask);
    return false;
}

void HWT101_startReceive(void)
{
    g_receiveEnabled = true;
    g_rxCnt = 0;
    g_parseState = HWT101_STATE_WAIT_HEADER;
}

void HWT101_stopReceive(void)
{
    g_receiveEnabled = false;
}

uint32_t HWT101_getRxByteCount(void)
{
    return g_rxByteCount;
}

uint32_t HWT101_getFrameCount(void)
{
    return g_frameCount;
}

uint32_t HWT101_getHeaderCount(void)
{
    return g_headerCount;
}

uint32_t HWT101_getTypeMatchCount(void)
{
    return g_typeMatch;
}

void HWT101_getFrameTypeCounts(uint32_t *outA, uint32_t *outG, uint32_t *outY)
{
    if (outA) *outA = g_typeCountA;
    if (outG) *outG = g_typeCountG;
    if (outY) *outY = g_typeCountY;
}
