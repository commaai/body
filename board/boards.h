extern hall_sensor hall_left;
extern hall_sensor hall_right;
uint32_t can_addr_offset;

void board_detect(void) {
  hw_type = board_id();
  // 0 = base, 1 = knee
  if (hw_type == HW_TYPE_BASE) {
    hall_left.hall_portA = GPIOC;
    hall_left.hall_pinA = GPIO_PIN_13;
    hall_left.hall_portB = GPIOC;
    hall_left.hall_pinB = GPIO_PIN_14;
    hall_left.hall_portC = GPIOC;
    hall_left.hall_pinC = GPIO_PIN_15;

    hall_right.hall_portA = GPIOC;
    hall_right.hall_pinA = GPIO_PIN_10;
    hall_right.hall_portB = GPIOC;
    hall_right.hall_pinB = GPIO_PIN_11;
    hall_right.hall_portC = GPIOC;
    hall_right.hall_pinC = GPIO_PIN_12;

    can_addr_offset = 0x0U;

    MX_GPIO_LED_Base_Init();
  } else if (hw_type == HW_TYPE_KNEE) {
    hall_left.hall_portA = GPIOC;
    hall_left.hall_pinA = GPIO_PIN_14;
    hall_left.hall_portB = GPIOC;
    hall_left.hall_pinB = GPIO_PIN_15;
    hall_left.hall_portC = GPIOC;
    hall_left.hall_pinC = GPIO_PIN_13;

    hall_right.hall_portA = GPIOD;
    hall_right.hall_pinA = GPIO_PIN_2;
    hall_right.hall_portB = GPIOC;
    hall_right.hall_pinB = GPIO_PIN_0;
    hall_right.hall_portC = GPIOC;
    hall_right.hall_pinC = GPIO_PIN_1;

    can_addr_offset = 0x100U;

    MX_SPI3_Init();
  } else {
    // Fail to detect, halt
    while(1) {}
  }
}
