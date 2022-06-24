#define SENSOR_COUNT 2

// Default addresses for AS5048B
#define AS5048_ADDRESS_LEFT 0x40
#define AS5048_ADDRESS_RIGHT 0x41
#define AS5048B_PROG_REG 0x03
#define AS5048B_ADDR_REG 0x15
#define AS5048B_ZEROMSB_REG 0x16 //bits 0..7
#define AS5048B_ZEROLSB_REG 0x17 //bits 0..5
#define AS5048B_GAIN_REG 0xFA
#define AS5048B_DIAG_REG 0xFB
#define AS5048B_MAGNMSB_REG 0xFC //bits 0..7
#define AS5048B_MAGNLSB_REG 0xFD //bits 0..5
#define AS5048B_ANGLMSB_REG 0xFE //bits 0..7
#define AS5048B_ANGLLSB_REG 0xFF //bits 0..5

extern I2C_HandleTypeDef hi2c1;


void angle_sensor_read(uint16_t *sensor_angle) {
  uint8_t buf[2];
  if (HAL_I2C_Mem_Read(&hi2c1, (AS5048_ADDRESS_LEFT<<1), AS5048B_ANGLMSB_REG, I2C_MEMADD_SIZE_8BIT, buf, 2, 5) == HAL_OK) {
    sensor_angle[0] = (buf[0] << 6) | (buf[1] & 0x3F);
  }
  if (HAL_I2C_Mem_Read(&hi2c1, (AS5048_ADDRESS_RIGHT<<1), AS5048B_ANGLMSB_REG, I2C_MEMADD_SIZE_8BIT, buf, 2, 5) == HAL_OK) {
    sensor_angle[1] = (buf[0] << 6) | (buf[1] & 0x3F);
  }
}
