/************************************************************************************

Filename    :   mpu6500.c
Content     :   Invensense MPU-6500 gyro/accel interface
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "mpu6500.h"
#include "mpu6500_register_map.h"
#include "spi.h"
#include "hw_config.h"
#include "gpio.h"
#include "timestamper.h"
#include "math.h"

#define R(a) (0x80 | a)

#define MPU_DEBUG 0
#define M_PI (3.14159265358979323846F)
//// This is gravity in Irvine in 10^-4m/s^2
//#define GRAVITY (97959)
// Gravity in Dongguan
#define GRAVITY (97859)
#define CONVERT_FS(fs) (10000.0F * M_PI / (fs * 180.0F))
#define UNSIGNED_COMPARE(a, b) ((a != b) && (a - b < INT32_MAX))

// This is a power of two to make the modulo just a bitwise AND
#define MPU_BUFFER_SLOTS 4
#define SLOT_FOR_FRAME(x) ((x) & (MPU_BUFFER_SLOTS - 1))

typedef struct mpu_time_struct {
    uint32_t timestamp;
    uint32_t count;
} mpu_time_t, *mpu_time_p;

// Use a ring buffer to store vsyncs to avoid memory access issues between
// the main thread and the interrupt that stores the data
static mpu_time_t mpu_buffer[MPU_BUFFER_SLOTS] = {{0}};

#define GYRO_NUM_RANGES 4
static const uint16_t gyro_ranges[GYRO_NUM_RANGES] = {250, 500, 1000, 2000};
static const uint8_t gyro_bitmasks[GYRO_NUM_RANGES] = {FS_SEL_250, FS_SEL_500, FS_SEL_1000, FS_SEL_2000};
static const float gyro_conversions[GYRO_NUM_RANGES] = {CONVERT_FS(FS_SCALE_250), 
    CONVERT_FS(FS_SCALE_500), CONVERT_FS(FS_SCALE_1000), CONVERT_FS(FS_SCALE_2000)};

#define ACCEL_NUM_RANGES 4
static const uint8_t accel_ranges[ACCEL_NUM_RANGES] = {2, 4, 8, 16};
static const uint8_t accel_bitmasks[ACCEL_NUM_RANGES] = {AFS_SEL_2, AFS_SEL_4, AFS_SEL_8, AFS_SEL_16};
static const float accel_conversions[ACCEL_NUM_RANGES] = {GRAVITY / AFS_SCALE_2, GRAVITY / AFS_SCALE_4, GRAVITY / AFS_SCALE_8, GRAVITY / AFS_SCALE_16};

typedef struct mpu6500_struct {
    spi_t spi;
    uint32_t spi_speed;
    uint32_t last_register_write;
    volatile uint32_t data_ready;
    uint32_t data_ready_read;
    uint8_t gyro_range;
    uint8_t accel_range;
    bool suspended;
} mpu6500_t, *mpu6500_p;

static mpu6500_t g_mpu6500 = {{0}};

void mpu6500_init(void)
{
#ifdef GYRO_FSYNC_PIN
    // We don't use FSYNC, so just keep it from floating
    gpio_config(GYRO_FSYNC_PORT, GYRO_FSYNC_PIN, GPIO_IN_PD);
#endif /* GYRO_FSYNC_PIN */
    
#ifdef GYRO_IRQ_TIM
    // Enable input capture of the gyro interrupt
    gpio_config(GYRO_IRQ_PORT, GYRO_IRQ_PIN, GPIO_AF);
    GPIO_PinAFConfig(GYRO_IRQ_PORT, GYRO_IRQ_SOURCE, GYRO_IRQ_AF);
    
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICInitStructure.TIM_Channel = GYRO_IRQ_CHANNEL;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    // We need to divide 8 kHz raw interrupt rate down to a 1 kHz sample rate
    // Conveniently the input capture on the interrupt can do this for us,
    // since the MPU-6500 can't internally on the DLPF settings we use.
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;
    TIM_ICInit(GYRO_IRQ_TIM, &TIM_ICInitStructure);
    
    // NVIC is already set up for this timer in timestamper_init, so don't reinit
    
    TIM_ClearITPendingBit(GYRO_IRQ_TIM, GYRO_IRQ_FLAG);
    TIM_ITConfig(GYRO_IRQ_TIM, GYRO_IRQ_FLAG, ENABLE);
#endif /* GYRO_IRQ_TIM */
    
    // Set the pins and ports for the gyro/accel spi interface
    g_mpu6500.spi.spi_port = MEMS_SPI_NUM;
    g_mpu6500.spi.gpio_port = MEMS_PORT;
    g_mpu6500.spi.ss_port = GYRO_CS_PORT;
    g_mpu6500.spi.sck_pin = MEMS_SCK_PIN;
    g_mpu6500.spi.sdo_pin = MEMS_MISO_PIN;
    g_mpu6500.spi.sdi_pin = MEMS_MOSI_PIN;
    g_mpu6500.spi.ss_pin = GYRO_CS_PIN;
    g_mpu6500.spi.sck_source = MEMS_SCK_SOURCE;
    g_mpu6500.spi.sdo_source = MEMS_MISO_SOURCE;
    g_mpu6500.spi.sdi_source = MEMS_MOSI_SOURCE;
    g_mpu6500.spi.rcc = MEMS_SPI_RCC;
    g_mpu6500.spi.af = MEMS_SPI_AF;
    // Use DMA for Tx and Rx
    g_mpu6500.spi.dma_rx_channel = MEMS_SPI_DMA_RX;
    g_mpu6500.spi.dma_rx_flag = MEMS_SPI_DMA_RX_FLAG;
    g_mpu6500.spi.dma_tx_channel = MEMS_SPI_DMA_TX;
    g_mpu6500.spi.dma_tx_flag = MEMS_SPI_DMA_TX_FLAG;
    
    // Initialize at 1 MHz for configuration of registers
    spi_init(&g_mpu6500.spi, MEMS_SPI_CONFIG_SPEED, 1);
    g_mpu6500.spi_speed = MEMS_SPI_CONFIG_SPEED;
    
    // make sure 100 ms has passed for the registers to stabilize
    // reset the device state
    mpu6500_set_register(PWR_MGMT_1, DEVICE_RESET);
    
    timestamper_delay_us(100000);
    
    // When using SPI, the signal path also needs to be reset
    mpu6500_set_register(SIGNAL_PATH_RESET, TEMP_RESET | GYRO_RESET | ACCEL_RESET);
    
    timestamper_delay_us(100000);
    
    // shut off I2C first since we use SPI
    mpu6500_set_register(USER_CTRL, I2C_IF_DIS);
    
    // set the internal clock to run off one of the gyro axes, also setting SLEEP to 0
    mpu6500_set_register(PWR_MGMT_1, CLKSEL_GYRO_Y);
    
    // set the gyro to 8kHz, low pass filter to 250 Hz
    mpu6500_set_register(INV_CONFIG, DLPF_CFG_256);
    
    // set the accel to 4kHz sample rate before divider
    mpu6500_set_register(ACCEL_CONFIG2, ACCEL_FCHOICE_B);
    
    // Run at 2000 degrees per second
    mpu6500_set_register(GYRO_CONFIG, FS_SEL_2000);
    g_mpu6500.gyro_range = 3;
    
    // Accel runs at 4G
    mpu6500_set_register(ACCEL_CONFIG, AFS_SEL_4);
    g_mpu6500.accel_range = 1;
    
    // Clear interrupt on any read
    mpu6500_set_register(INT_PIN_CFG, INT_RD_CLEAR);
    
    // Turn on data ready interrupts
    mpu6500_set_register(INT_ENABLE, DATA_RDY_EN);
    
    // Switch to 8 MHz for normal sensor data reading
    // TODO: Should we switch to 16 MHz?  The MPU-6500 can handle 20 MHz,
    // but the LIS3MDL can only do 10 MHz
    spi_reconfigure(&g_mpu6500.spi, MEMS_SPI_READ_SPEED);
    g_mpu6500.spi_speed = MEMS_SPI_READ_SPEED;
}

void mpu6500_sleep(bool sleep)
{
    if (sleep) {
        g_mpu6500.suspended = 1;
        
        // Sleep, which shuts off 
        mpu6500_set_register(PWR_MGMT_1, SLEEP);
    } else {
        // Go out of sleep mode
        mpu6500_set_register(PWR_MGMT_1, CLKSEL_GYRO_Y);
        
        g_mpu6500.suspended = 0;
    }
}

bool mpu6500_motion_interrupt(void)
{
    // TODO: Implement wake from motion, #16
    return 0;
}

static inline int32_t convert_accel(int16_t accel)
{
    return lroundf((float)accel * accel_conversions[g_mpu6500.accel_range]);
}

// TODO: Should we also deal with the gyro in fixed point?
static inline float convert_gyro(int16_t gyro)
{
    return (float)gyro * gyro_conversions[g_mpu6500.gyro_range];
}

static inline int16_t convert_temp(int16_t temp)
{
    // Convert from the internal representation to centidegrees
    return 10000 * (int32_t)temp / 33387 + 2100;
}

bool mpu6500_read(mpu6500_data_p data, bool raw)
{
    // Only read if there is new data
    bool new_data = UNSIGNED_COMPARE(g_mpu6500.data_ready, g_mpu6500.data_ready_read);
    if ((!new_data) || g_mpu6500.suspended)
        return 0;

    // We only have real data for the most recent timestamp, so use that
    // TODO: Use the MPU-6500's FIFO to not lose any samples?
    if (new_data)
        g_mpu6500.data_ready_read = g_mpu6500.data_ready;
    
    mpu_time_p time = &mpu_buffer[SLOT_FOR_FRAME(g_mpu6500.data_ready_read)];
    // TODO: Check if count mismatches data_ready_read to see if we fell behind
    data->count = time->count;
    data->timestamp = time->timestamp;
    
    static const uint8_t command[15] = {R(ACCEL_XOUT_H)};
    uint8_t buf[15];
    spi_transfer(&g_mpu6500.spi, command, buf, sizeof(command), 1);

    // Start 1 byte in since the command covered the first byte
    int16_t ax = (buf[1] << 8) | buf[2];
    int16_t ay = (buf[3] << 8) | buf[4];
    int16_t az = (buf[5] << 8) | buf[6];
    int16_t t = (buf[7] << 8) | buf[8];
    int16_t gx = (buf[9] << 8) | buf[10];
    int16_t gy = (buf[11] << 8) | buf[12];
    int16_t gz = (buf[13] << 8) | buf[14];

    // Use OVR Headset coordinates from the lowest level
    // That is, Z-in, Y-up, X-right from the user with the headset on
    if (raw) {
        // Negate Y and Z
        data->accel[0] = ax;
        data->accel[1] = -ay;
        data->accel[2] = -az;
        data->temperature = t;
        data->gyro[0] = (float)gx;
        data->gyro[1] = (float)-gy;
        data->gyro[2] = (float)-gz;
    } else {
        // Negate Y and Z
        data->accel[0] = convert_accel(ax);
        data->accel[1] = convert_accel(-ay);
        data->accel[2] = convert_accel(-az);
        data->temperature = convert_temp(t);
        data->gyro[0] = convert_gyro(gx);
        data->gyro[1] = convert_gyro(-gy);
        data->gyro[2] = convert_gyro(-gz);
    }

    return 1;
}

uint8_t mpu6500_closest_accel_range(uint8_t accel)
{
    uint8_t i = ACCEL_NUM_RANGES - 1;
    uint8_t target_accel = accel_ranges[i];
    // Grab the smallest range that will include the desired range
    while (i--) {
        if (accel_ranges[i] < accel) break;
        target_accel = accel_ranges[i];
    }
    return target_accel;
}

uint16_t mpu6500_closest_gyro_range(uint16_t gyro)
{
    uint8_t i = GYRO_NUM_RANGES - 1;
    uint16_t target_gyro = gyro_ranges[i];
    while (i--) {
        if (gyro_ranges[i] < gyro) break;
        target_gyro = gyro_ranges[i];
    }
    return target_gyro;
}

void mpu6500_set_ranges(uint8_t accel, uint16_t gyro)
{
    if (g_mpu6500.suspended)
        return;

    uint8_t i;
    // find the correct index for the value passed in
    for (i = 0; i < ACCEL_NUM_RANGES; i++) {
        if (accel == accel_ranges[i]) {
            // only set if the value is actually different
            if (g_mpu6500.accel_range != i) {
                g_mpu6500.accel_range = i;
                mpu6500_set_register(ACCEL_CONFIG, accel_bitmasks[g_mpu6500.accel_range]);
            }
            break;
        }
    }

    for (i = 0; i < GYRO_NUM_RANGES; i++) {
        if (gyro == gyro_ranges[i]) {
            if (g_mpu6500.gyro_range != i) {
                g_mpu6500.gyro_range = i;
                mpu6500_set_register(GYRO_CONFIG, gyro_bitmasks[g_mpu6500.gyro_range]);
            }
            break;
        }
    }
}

void mpu6500_get_ranges(uint8_t *accel, uint16_t *gyro)
{
    *accel = accel_ranges[g_mpu6500.accel_range];
    *gyro = gyro_ranges[g_mpu6500.gyro_range];
}

void mpu6500_set_register(uint8_t reg, uint8_t payload)
{
    uint32_t now = timestamper_get_time();
    uint32_t diff = now - g_mpu6500.last_register_write;
    if (diff < 1000) {
        // We need to keep at least 1 ms between register sets on the MPU-6500
        timestamper_delay_us(1000 - diff);
    }
    g_mpu6500.last_register_write = now;
    
    // MPU6500 has a slower speed for configuration registers, so switch to it
    // if needed
    bool reconfigured_spi = 0;
    if (g_mpu6500.spi_speed != MEMS_SPI_CONFIG_SPEED) {
        spi_reconfigure(&g_mpu6500.spi, MEMS_SPI_CONFIG_SPEED);
        reconfigured_spi = 1;
    }
    
    // TODO: We need to switch back to 1 MHz if we're higher on SPI
    uint8_t set_reg[] = {reg, payload};
    uint8_t rx_buf[2];
    spi_transfer(&g_mpu6500.spi, set_reg, rx_buf, sizeof(set_reg), 0);
    
#if MPU_DEBUG
    // Read back verification
    uint8_t command[2] = {R(reg), 0};
    spi_transfer(&g_mpu6500.spi, command, rx_buf, sizeof(rx_buf), 0);
    if ((rx_buf[1] != payload) && (reg != PWR_MGMT_1))
        while (1);
#endif /* MPU_DEBUG */
    
    // Put the SPI port back to high speed if we started that way
    if (reconfigured_spi) {
        spi_reconfigure(&g_mpu6500.spi, MEMS_SPI_READ_SPEED);
    }
}

void mpu6500_ready(bool rolled)
{
    // Grab the buffer to store gyro information hopefully atomically
    mpu_time_p time = &mpu_buffer[SLOT_FOR_FRAME(g_mpu6500.data_ready + 1)];
    time->count = g_mpu6500.data_ready + 1;
#ifdef GYRO_IRQ_PIN
    // Get the high 16 bits from the 32 bit timer
    uint32_t now = timestamper_get_time();
    // Get the low 16 bits directly from the capture
    time->timestamp = (now & 0xFFFF0000) | TIM_GetCapture3(GYRO_IRQ_TIM);
    
    // If the capture value is greater than the current timer value
    // and we know that the timer overflow counter was incremented,
    // it means the capture happened before the roll and the overflow increment
    // needs to be removed.
    if (rolled && ((now & 0xFFFF) < (time->timestamp & 0xFFFF))) {
        time->timestamp -= 0x10000;
    }
#endif /* GYRO_IRQ_PIN */
    
    g_mpu6500.data_ready++;
}
