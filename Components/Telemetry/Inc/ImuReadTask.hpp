/*
 * ImuReadTask.hpp
 *
 * Imu read task (.hpp): Declares functions for retrieving accelerometer
 * and gyro data from fifo.
 *
 */

void imu_read_task(void);
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
