#include "icm42605.h"
#include "los_task.h"

ICM_Raw_Data icm_raw_data;

float accel_sensitivity = 0.244f;  //加速度的最小分辨率 mg/LSB
float gyro_sensitivity = 32.8f;  //陀螺仪的最小分辨率

#if defined LOSCFG_ICM42605_USE_HARD_SPI
extern SPI_HandleTypeDef hspi2;

inline void icm_cs_low() {
    HAL_GPIO_WritePin(ICM_PORT_CS, ICM_PIN_CS, GPIO_PIN_RESET);
}

inline void icm_cs_high() {
    HAL_GPIO_WritePin(ICM_PORT_CS, ICM_PIN_CS, GPIO_PIN_SET);
}

uint8_t icm_swap_byte(uint8_t tx_data) {
    uint8_t rx_data = 0;
    HAL_SPI_TransmitReceive(&hspi2, &tx_data, &rx_data, 1, 5);
    return rx_data;
}

void icm_swap_data(uint8_t* buf, uint8_t len) {
    for (int i = 0; i < len; i++) {
        *buf = icm_swap_byte(*buf);
        buf++;
    }
}
#elif defined LOSCFG_ICM42605_USE_HARD_I2C
extern I2C_HandleTypeDef hi2c1;
#endif

uint8_t icm_read_reg(uint8_t reg) {
    uint8_t reg_val = 0;

#if defined LOSCFG_ICM42605_USE_HARD_SPI
    uint8_t first_bit = reg | 0x80; // 读寄存器时，第一个字节第一位为1

    icm_cs_low();
    icm_swap_data(&first_bit, 1);
    icm_swap_data(&reg_val, 1);
    icm_cs_high();
#elif defined LOSCFG_ICM42605_USE_HARD_I2C
    HAL_I2C_Mem_Read(&hi2c1, ICM_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, &reg_val, 1, 1000);
#endif
    return reg_val;
}

void icm_read_regs(uint8_t reg, uint8_t* buf, uint16_t len) {
#if defined LOSCFG_ICM42605_USE_HARD_SPI
    uint8_t first_bit = reg | 0x80;

    icm_cs_low();
    icm_swap_data(&first_bit, 1);
    icm_swap_data(buf, len);
    icm_cs_high();
#elif defined LOSCFG_ICM42605_USE_HARD_I2C
    HAL_I2C_Mem_Read(&hi2c1, ICM_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
#endif
}

void icm_write_reg(uint8_t reg, uint8_t data) {
#if defined LOSCFG_ICM42605_USE_HARD_SPI
    uint8_t first_bit = reg & 0x7f; // 写寄存器时，第一个字节第一位为0

    icm_cs_low();
    icm_swap_data(&first_bit, 1);
    icm_swap_data(&data, 1);
    icm_cs_high();
#elif defined LOSCFG_ICM42605_USE_HARD_I2C
    HAL_I2C_Mem_Write(&hi2c1, ICM_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
#endif
}

float icm_set_ares(uint8_t scale) {
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

    return accel_sensitivity;
}

float icm_set_gres(uint8_t scale) {
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

    return gyro_sensitivity;
}

int8_t icm_init(void) {
    uint8_t reg_val = 0;
    reg_val = icm_read_reg(ICM_WHO_AM_I);

    if (reg_val != ICM42605_ID)
        return -1;

    icm_write_reg(ICM_REG_BANK_SEL, 0); //设置bank 0区域寄存器
    icm_write_reg(ICM_REG_BANK_SEL, 0x01); //软复位传感器
    LOS_TaskDelay(100);

    icm_write_reg(ICM_REG_BANK_SEL, 1); //设置bank 1区域寄存器
#if defined LOSCFG_ICM42605_USE_HARD_SPI
    icm_write_reg(ICM_INTF_CONFIG4, 0x02); //设置为4线SPI通信
#elif defined LOSCFG_ICM42605_USE_HARD_I2C
    icm_write_reg(ICM_INTF_CONFIG4, 0x00);
#endif
    icm_write_reg(ICM_REG_BANK_SEL, 0); //设置bank 0区域寄存器
    icm_write_reg(ICM_FIFO_CONFIG, 0x40); //Stream-to-FIFO Mode(page61)

    reg_val = icm_read_reg(ICM_INT_SOURCE0);
    icm_write_reg(ICM_INT_SOURCE0, 0x00);
    icm_write_reg(ICM_FIFO_CONFIG2, 0x00); // watermark
    icm_write_reg(ICM_FIFO_CONFIG3, 0x02); // watermark
    icm_write_reg(ICM_INT_SOURCE0, reg_val);
    icm_write_reg(ICM_FIFO_CONFIG1, 0x63); // Enable the accel and gyro to the FIFO

    icm_write_reg(ICM_REG_BANK_SEL, 0x00);
    icm_write_reg(ICM_INT_CONFIG, 0x36);

    icm_write_reg(ICM_REG_BANK_SEL, 0x00);
    reg_val = icm_read_reg(ICM_INT_SOURCE0);
    reg_val |= (1 << 2); //FIFO_THS_INT1_ENABLE
    icm_write_reg(ICM_INT_SOURCE0, reg_val);

    icm_set_ares(AFS_8G);
    icm_write_reg(ICM_REG_BANK_SEL, 0x00);
    reg_val = icm_read_reg(ICM_ACCEL_CONFIG0);//page74
    reg_val |= (AFS_8G << 5);   //量程 ±8g
    reg_val |= (AODR_50Hz);     //输出速率 50HZ
    icm_write_reg(ICM_ACCEL_CONFIG0, reg_val);

    icm_set_gres(GFS_1000DPS);
    icm_write_reg(ICM_REG_BANK_SEL, 0x00);
    reg_val = icm_read_reg(ICM_GYRO_CONFIG0);//page73
    reg_val |= (GFS_1000DPS << 5);   //量程 ±1000dps
    reg_val |= (GODR_50Hz);     //输出速率 50HZ
    icm_write_reg(ICM_GYRO_CONFIG0, reg_val);

    icm_write_reg(ICM_REG_BANK_SEL, 0x00);
    reg_val = icm_read_reg(ICM_PWR_MGMT0); //读取PWR—MGMT0当前寄存器的值(page72)
    reg_val &= ~(1 << 5);//使能温度测量
    reg_val |= ((3) << 2);//设置GYRO_MODE  0:关闭 1:待机 2:预留 3:低噪声
    reg_val |= (3);//设置ACCEL_MODE 0:关闭 1:关闭 2:低功耗 3:低噪声
    icm_write_reg(ICM_PWR_MGMT0, reg_val);
    LOS_UDelay(200); //操作完PWR—MGMT0寄存器后 200us内不能有任何读写寄存器的操作

    return 0;
}

float icm_get_temperature() {
    uint8_t buf[2] = {0};
    icm_read_regs(ICM_TEMP_DATA1, buf, 2);

    return (int16_t)((buf[0] << 8) | buf[1]) / 132.48 + 25;
}

void icm_get_raw_data(ICM_Raw_Data* icm) {
    uint8_t buf[12] = {0};
    icm_read_regs(ICM_ACCEL_DATA_X1, buf, 12);
    icm->accel_x = (int16_t)((buf[0] << 8) | buf[1]) * accel_sensitivity;
    icm->accel_y = (int16_t)((buf[2] << 8) | buf[3]) * accel_sensitivity;
    icm->accel_z = (int16_t)((buf[4] << 8) | buf[5]) * accel_sensitivity;
    icm->angular_x = (int16_t)((buf[6] << 8) | buf[7]) * gyro_sensitivity;
    icm->angular_y = (int16_t)((buf[8] << 8) | buf[9]) * gyro_sensitivity;
    icm->angular_z = (int16_t)((buf[10] << 8) | buf[11]) * gyro_sensitivity;
}
