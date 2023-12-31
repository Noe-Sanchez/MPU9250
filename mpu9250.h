/**
 * mpu9250.h
 *
 * STM32 library for the MPU9250 IMU.
 *
 * Author: Noe Sanchez
 * Date: 2023-06-23
 *
 * Exported functions:
 *   mpu9250_update_accel_gyro() - Updates the accel and gyro values.
 *   mpu9250_write_reg() - Writes a value to a register.
 *   mpu9250_read_reg() - Reads a value from a register.
 * 
 * Exported struct definitions:
 *  mpu9250 - Struct containing the IMU values and SPI configuration.
 * 
 * Exported enums:
 *  mpu9250_accel_resolution - Accelerometer resolution.
 *  mpu9250_gyro_resolution - Gyroscope resolution.
 * 
 * License: GPL v3. See LICENSE file for details.
 * 
 */

enum mpu9250_accel_resolution{
  ACCEL_RESOLUTION_2G,
  ACCEL_RESOLUTION_4G,
  ACCEL_RESOLUTION_8G,
  ACCEL_RESOLUTION_16G,
};

enum mpu9250_gyro_resolution{
  GYRO_RESOLUTION_250DPS,
  GYRO_RESOLUTION_500DPS,
  GYRO_RESOLUTION_1000DPS,
  GYRO_RESOLUTION_2000DPS,
};

enum mpu9250_mag_resolution{
  MAG_RESOLUTION_14BITS,
  MAG_RESOLUTION_16BITS,
};

typedef struct mpu9250{
  float accel_x;
  float accel_y;
  float accel_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float mag_x;
  float mag_y;
  float mag_z;
  float mag_x_bias;
  float mag_y_bias;
  float mag_z_bias;
  float temp;
  float accel_resolution;
  float gyro_resolution;
  float mag_resolution;
  GPIO_TypeDef* cs_gpio_port;
  uint16_t cs_gpio_pin;
  SPI_HandleTypeDef* hspi;
} mpu9250;

void mpu9250_update_accel_gyro(mpu9250* mpu);
void mpu9250_write_reg( mpu9250* mpu, uint8_t reg, uint8_t data);
void mpu9250_read_register( mpu9250* mpu, uint8_t address, uint8_t* buffer, uint8_t len);
void mpu9250_set_accel_resolution(mpu9250* mpu, enum mpu9250_accel_resolution res);
void mpu9250_set_gyro_resolution(mpu9250* mpu, enum mpu9250_gyro_resolution res);

void mpu9250_write_register( mpu9250* mpu, uint8_t address, uint8_t data){
	HAL_GPIO_WritePin(mpu->cs_gpio_port, mpu->cs_gpio_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(mpu->hspi, &address, 1, 100);
	HAL_SPI_Transmit(mpu->hspi, &data, 1, 100);
	HAL_GPIO_WritePin(mpu->cs_gpio_port, mpu->cs_gpio_pin, GPIO_PIN_SET);
}
void mpu9250_read_register( mpu9250* mpu, uint8_t address, uint8_t* buffer, uint8_t len){
	uint8_t _address = 0x80|address; // Set MSB to 1 for reading
	HAL_GPIO_WritePin(mpu->cs_gpio_port, mpu->cs_gpio_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(mpu->hspi, &_address , 1, 100);
	HAL_SPI_Receive(mpu->hspi, buffer, len, 100);
	HAL_GPIO_WritePin(mpu->cs_gpio_port, mpu->cs_gpio_pin, GPIO_PIN_SET);
}

void mpu9250_update_accel_gyro( mpu9250* mpu ){
  uint8_t buffer[14];
  mpu9250_read_register(mpu, 59, buffer, 14);
  mpu->accel_x = ((float)(( (int16_t)buffer[0]  <<8) | (int16_t)buffer[1]) ) * mpu->accel_resolution;
  mpu->accel_y = ((float)(( (int16_t)buffer[2]  <<8) | (int16_t)buffer[3]) ) * mpu->accel_resolution;
  mpu->accel_z = ((float)(( (int16_t)buffer[4]  <<8) | (int16_t)buffer[5]) ) * mpu->accel_resolution;
  mpu->temp    = ((float)(( (int16_t)buffer[6]  <<8) | (int16_t)buffer[7]) )  / 333.87 + 21.0;
  mpu->gyro_x  = ((float)(( (int16_t)buffer[8]  <<8) | (int16_t)buffer[9]) ) * mpu->gyro_resolution;
  mpu->gyro_y  = ((float)(( (int16_t)buffer[10] <<8) | (int16_t)buffer[11])) * mpu->gyro_resolution;
  mpu->gyro_z  = ((float)(( (int16_t)buffer[12] <<8) | (int16_t)buffer[13])) * mpu->gyro_resolution;
}

void mpu9250_update_mag( mpu9250* mpu ){
  uint8_t buffer[7];
  mpu9250_read_register(mpu, 0x03, buffer, 7);
  mpu->mag_x = ((float)(( (int16_t)buffer[1]  <<8) | (int16_t)buffer[0]) ) * mpu->mag_resolution * mpu->mag_x_bias;
  mpu->mag_y = ((float)(( (int16_t)buffer[3]  <<8) | (int16_t)buffer[2]) ) * mpu->mag_resolution * mpu->mag_y_bias;
  mpu->mag_z = ((float)(( (int16_t)buffer[5]  <<8) | (int16_t)buffer[4]) ) * mpu->mag_resolution * mpu->mag_z_bias;
}

void mpu9250_set_accel_resolution(mpu9250* mpu, enum mpu9250_accel_resolution res){
  uint8_t current_config;
  mpu9250_read_register(mpu, 28, &current_config, 1);
  current_config &= ~0x18;
  switch (res){
    case ACCEL_RESOLUTION_2G:
      mpu->accel_resolution = 2.0/32768.0;
      mpu9250_write_register(mpu, 28, current_config | (0x00 << 3));
      break;
    case ACCEL_RESOLUTION_4G:
      mpu->accel_resolution = 4.0/32768.0;
      mpu9250_write_register(mpu, 28, current_config | (0x01 << 3));
      break;
    case ACCEL_RESOLUTION_8G:
      mpu->accel_resolution = 8.0/32768.0;
      mpu9250_write_register(mpu, 28, current_config | (0x10 << 3));
      break;
    case ACCEL_RESOLUTION_16G:
      mpu->accel_resolution = 16.0/32768.0;
      mpu9250_write_register(mpu, 28, current_config | (0x11 << 3));
      break;
    default:
      mpu->accel_resolution = 2.0/32768.0;
      mpu9250_write_register(mpu, 28, current_config | (0x00 << 3));
      break; 
  }
}

void mpu9250_set_gyro_resolution(mpu9250* mpu, enum mpu9250_gyro_resolution res){
  uint8_t current_config;
  mpu9250_read_register(mpu, 27, &current_config, 1);
  current_config &= ~0x18;
  switch (res){
    case GYRO_RESOLUTION_250DPS:
      mpu->gyro_resolution = 250.0/32768.0;
      mpu9250_write_register(mpu, 27, current_config | (0x00 << 3));
      break;
    case GYRO_RESOLUTION_500DPS:
      mpu->gyro_resolution = 500.0/32768.0;
      mpu9250_write_register(mpu, 27, current_config | (0x01 << 3));
      break;
    case GYRO_RESOLUTION_1000DPS:
      mpu->gyro_resolution = 1000.0/32768.0;
      mpu9250_write_register(mpu, 27, current_config | (0x10 << 3));
      break;
    case GYRO_RESOLUTION_2000DPS:
      mpu->gyro_resolution = 2000.0/32768.0;
      mpu9250_write_register(mpu, 27, current_config | (0x11 << 3));
      break;
    default:
      mpu->gyro_resolution = 250.0/32768.0;
      mpu9250_write_register(mpu, 27, current_config | (0x00 << 3));
      break; 
  }
}

void mpu9250_set_mag_resolution(mpu9250* mpu, enum mpu9250_mag_resolution res){
  mpu9250_write_register(mpu, 0x0A, 0x00); // Power down
  HAL_Delay(10);
  mpu9250_write_register(mpu, 0x0A, 0x0F); // Fuse ROM
  HAL_Delay(10);
  uint8_t buffer[3];
  mpu9250_read_register(mpu, 0x10, &buffer[0], 3); 
  mpu->mag_x_bias = (float)(( buffer[0] - 128 ) / 256. + 1.); 
  mpu->mag_y_bias = (float)(( buffer[1] - 128 ) / 256. + 1.); 
  mpu->mag_z_bias = (float)(( buffer[2] - 128 ) / 256. + 1.);
  mpu9250_write_register(mpu, 0x0A, 0x00); // Power down
  HAL_Delay(10); 

  uint8_t current_config;
  mpu9250_read_register(mpu, 0x0A, &current_config, 1);
  switch (res){
    case MAG_RESOLUTION_14BITS:
      mpu->mag_resolution = 10.*4912.0/8190.0;
      mpu9250_write_register(mpu, 0x0A, current_config & ~(0x01 << 4));
      break;
    case MAG_RESOLUTION_16BITS:
      mpu->mag_resolution = 10.*4912.0/32760.0;
      mpu9250_write_register(mpu, 0x0A, current_config | (0x01 << 4));
      break;
    default:
      mpu->mag_resolution = 10.*4912.0/8190.0;
      mpu9250_write_register(mpu, 0x0A, current_config & ~(0x01 << 4));
      break; 
  }
}