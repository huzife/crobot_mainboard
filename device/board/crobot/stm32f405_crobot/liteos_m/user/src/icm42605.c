#include "icm42605.h"
#include "icm42605_reg.h"
#include "los_mux.h"
#include "los_task.h"

#if defined LOSCFG_ICM42605_USE_HARD_SPI
#include "spi.h"
#define ICM42605_CS_LOW() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET)
#define ICM42605_CS_HIGH() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET)
// TODO: init spi
static SPI_HandleTypeDef icm42605_spi;
static void icm42605_spi_init() {}
#elif defined LOSCFG_ICM42605_USE_HARD_I2C
static I2C_HandleTypeDef icm42605_i2c;
static void icm42605_i2c_init() {
    // msp init
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    __HAL_RCC_I2C1_CLK_ENABLE();

    // hal i2c init
    icm42605_i2c.Instance = I2C1;
    icm42605_i2c.Init.ClockSpeed = 100000;
    icm42605_i2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
    icm42605_i2c.Init.OwnAddress1 = 0;
    icm42605_i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    icm42605_i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    icm42605_i2c.Init.OwnAddress2 = 0;
    icm42605_i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    icm42605_i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&icm42605_i2c);
}
#endif

static float accel_sensitivity;
static float gyro_sensitivity;
static uint32_t imu_data_mtx;
static float imu_temperature;
static IMU_Data imu_data;

static uint8_t read_reg(uint8_t reg) {
    uint8_t val;
#if defined LOSCFG_ICM42605_USE_HARD_SPI
    uint8_t first_bit = reg | 0x80;

    ICM42605_CS_LOW();
    HAL_SPI_Transmit(&hspi2, &first_bit, 1, 10);
    HAL_SPI_Receive(&hspi2, &val, 1, 10);
    ICM42605_CS_HIGH();
#elif defined LOSCFG_ICM42605_USE_HARD_I2C
    HAL_I2C_Mem_Read(&icm42605_i2c, ICM42605_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 10);
#endif
    return val;
}

static void read_regs(uint8_t reg, uint8_t* buf, uint16_t len) {
#if defined LOSCFG_ICM42605_USE_HARD_SPI
    uint8_t first_bit = reg | 0x80;

    ICM42605_CS_LOW();
    HAL_SPI_Transmit(&hspi2, &first_bit, 1, 10);
    HAL_SPI_Receive(&hspi2, buf, len, 100);
    ICM42605_CS_HIGH();
#elif defined LOSCFG_ICM42605_USE_HARD_I2C
    HAL_I2C_Mem_Read(&icm42605_i2c, ICM42605_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
#endif
}

static void write_reg(uint8_t reg, uint8_t data) {
#if defined LOSCFG_ICM42605_USE_HARD_SPI
    uint8_t first_bit = reg & 0x7f; // 写寄存器时，第一个字节第一位为0

    ICM42605_CS_LOW();
    HAL_SPI_Transmit(&hspi2, &first_bit, 1, 10);
    HAL_SPI_Transmit(&hspi2, &data, 1, 10);
    ICM42605_CS_HIGH();
#elif defined LOSCFG_ICM42605_USE_HARD_I2C
    HAL_I2C_Mem_Write(&icm42605_i2c, ICM42605_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
#endif
}

static void set_ares(uint8_t scale) {
    switch(scale) {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    case AFS_2G:
        accel_sensitivity = 2.0f / 32768.0f;
        break;
    case AFS_4G:
        accel_sensitivity = 4.0f / 32768.0f;
        break;
    case AFS_8G:
        accel_sensitivity = 8.0f / 32768.0f;
        break;
    case AFS_16G:
        accel_sensitivity = 16.0f / 32768.0f;
        break;
    }
}

static void set_gres(uint8_t scale) {
    switch(scale) {
    case GFS_15_125DPS:
        gyro_sensitivity = 15.125f / 32768.0f;
        break;
    case GFS_31_25DPS:
        gyro_sensitivity = 31.25f / 32768.0f;
        break;
    case GFS_62_5DPS:
        gyro_sensitivity = 62.5f / 32768.0f;
        break;
    case GFS_125DPS:
        gyro_sensitivity = 125.0f / 32768.0f;
        break;
    case GFS_250DPS:
        gyro_sensitivity = 250.0f / 32768.0f;
        break;
    case GFS_500DPS:
        gyro_sensitivity = 500.0f / 32768.0f;
        break;
    case GFS_1000DPS:
        gyro_sensitivity = 1000.0f / 32768.0f;
        break;
    case GFS_2000DPS:
        gyro_sensitivity = 2000.0f / 32768.0f;
        break;
    }
}

static void icm42605_config() {
    // reset
    write_reg(ICM42605_REG_BANK_SEL, 0);
    write_reg(ICM42605_DEVICE_CONFIG, 0x01);
    LOS_TaskDelay(100);

    // select bank 0
    write_reg(ICM42605_REG_BANK_SEL, 0);

    uint8_t val;
    // enable gyro and accel in low noise mode, enable temperature measurement
    val = read_reg(ICM42605_PWR_MGMT0);
    val &= ~(1 << 5); // temperature
    val |= 3; // accel mode
    val |= (3 << 2); // gyro mode
    write_reg(ICM42605_PWR_MGMT0, val);
    LOS_TaskDelay(1); // do not issue any register wirtes for 200us after change accel and gyro mode

    // set accel scale and data rate
    set_ares(AFS_8G);
    val = read_reg(ICM42605_ACCEL_CONFIG0);
    write_reg(ICM42605_ACCEL_CONFIG0, val | AODR_200Hz | (AFS_8G << 5));

    // set gyro scale and data rate
    set_gres(GFS_500DPS);
    val = read_reg(ICM42605_GYRO_CONFIG0);
    write_reg(ICM42605_GYRO_CONFIG0, val | GODR_200Hz | (GFS_500DPS << 5));

    // set temperature filter bandwidth and order of gyro UI filter
    val = read_reg(ICM42605_GYRO_CONFIG1);
    write_reg(ICM42605_GYRO_CONFIG1, val | 0xD0);
}

void icm42605_init() {
#if defined LOSCFG_ICM42605_USE_HARD_SPI
    icm42605_spi_init();
#elif defined LOSCFG_ICM42605_USE_HARD_I2C
    icm42605_i2c_init();
#endif
    icm42605_config();
    LOS_MuxCreate(&imu_data_mtx);
}

void icm42605_update_temperature() {
    uint8_t buf[2] = {0};
    read_regs(ICM42605_TEMP_DATA1, buf, 2);
    imu_temperature = (int16_t)((buf[0] << 8) | buf[1]) / 132.48f + 25;
}

void icm42605_update_data() {
    uint8_t buf[12] = {0};
    read_regs(ICM42605_ACCEL_DATA_X1, buf, 12);

    IMU_Data data;
    data.accel_x = (int16_t)((buf[0] << 8) | buf[1]) * accel_sensitivity;
    data.accel_y = (int16_t)((buf[2] << 8) | buf[3]) * accel_sensitivity;
    data.accel_z = (int16_t)((buf[4] << 8) | buf[5]) * accel_sensitivity;
    data.angular_x = (int16_t)((buf[6] << 8) | buf[7]) * gyro_sensitivity;
    data.angular_y = (int16_t)((buf[8] << 8) | buf[9]) * gyro_sensitivity;
    data.angular_z = (int16_t)((buf[10] << 8) | buf[11]) * gyro_sensitivity;

    LOS_MuxPend(imu_data_mtx, 10);
    imu_data.accel_x = -data.accel_y;
    imu_data.accel_y = data.accel_x;
    imu_data.accel_z = data.accel_z;
    imu_data.angular_x = -data.angular_x;
    imu_data.angular_y = data.angular_y;
    imu_data.angular_z = data.angular_z;
    LOS_MuxPost(imu_data_mtx);
}

float icm42605_get_temperature() {
    return imu_temperature;
}

IMU_Data icm42605_get_data() {
    LOS_MuxPend(imu_data_mtx, 10);
    IMU_Data data = imu_data;
    LOS_MuxPost(imu_data_mtx);

    return data;
}
