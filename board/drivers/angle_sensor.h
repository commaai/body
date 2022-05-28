#define SENSOR_COUNT 2 // Limit is 4! If more is needed - CAN message should be changed

#define SPI_TIMEOUT 2710

#define SPI_CMD_READ 0x4000U // Read command
#define SPI_REG_AGC 0x3FFDU // AGC register
#define SPI_REG_MAG 0x3FFEU // Magnitude register
#define SPI_REG_DATA 0x3FFFU // Angle register
#define SPI_REG_CLRERR 0x0001U // Clear error flag, should be used with read command
#define SPI_NOP 0x0000U // NOP, to read data on the next transfer
#define PARITY_BIT_SET 0x8000U

extern SPI_HandleTypeDef hspi3;

uint16_t angle_data[SENSOR_COUNT] = { 0 };

const uint16_t clear_error_cmd = (SPI_CMD_READ | SPI_REG_CLRERR);
const uint16_t read_angle_cmd = (PARITY_BIT_SET | SPI_CMD_READ | SPI_REG_DATA);
const uint16_t nop_cmd = SPI_NOP;


uint8_t spiCalcEvenParity(uint16_t value) {
  uint8_t cnt = 0;
  for (uint8_t i = 0; i < 16; i++) {
    if (value & 0x1) {
      cnt++;
    }
    value >>= 1;
  }
  return cnt & 0x1;
}

void angle_sensor_read(uint16_t *sensor_angle) {
  HAL_GPIO_WritePin(AS5048_CS_PORT, AS5048A_CS_PIN, GPIO_PIN_RESET);
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    HAL_SPI_Transmit(&hspi3, (uint8_t*)&read_angle_cmd,  1, SPI_TIMEOUT);
  }
  HAL_GPIO_WritePin(AS5048_CS_PORT, AS5048A_CS_PIN, GPIO_PIN_SET);
  HAL_Delay(1);

  HAL_GPIO_WritePin(AS5048_CS_PORT, AS5048A_CS_PIN, GPIO_PIN_RESET);
  for (int8_t i = (SENSOR_COUNT-1); i >= 0; i--) {
    HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&nop_cmd, (uint8_t*)(&angle_data[i]), 1, 2710);
  }
  HAL_GPIO_WritePin(AS5048_CS_PORT, AS5048A_CS_PIN, GPIO_PIN_SET);
  HAL_Delay(1);

  bool error_flag_set = false;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if ((angle_data[i] >> 15) == spiCalcEvenParity((angle_data[i] & 0x7fff))) {
      if (angle_data[i] & SPI_CMD_READ) {
        error_flag_set = true;
      } else {
        if ((sensor_angle[i] == 0) || (ABS(sensor_angle[i] - (angle_data[i] & 0x3fff)) < 200)) {
          sensor_angle[i] = (angle_data[i] & 0x3fff);
        }
      }
    }
  }

  if (error_flag_set) {
    HAL_GPIO_WritePin(AS5048_CS_PORT, AS5048A_CS_PIN, GPIO_PIN_RESET);
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
     HAL_SPI_Transmit(&hspi3, (uint8_t*)&clear_error_cmd, 1, SPI_TIMEOUT);
    }
    HAL_GPIO_WritePin(AS5048_CS_PORT, AS5048A_CS_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
  }
}
