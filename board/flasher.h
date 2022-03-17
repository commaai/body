typedef union {
  uint16_t w;
  struct BW {
    uint8_t msb;
    uint8_t lsb;
  }
  bw;
}
uint16_t_uint8_t;

typedef union _USB_Setup {
  uint32_t d8[2];
  struct _SetupPkt_Struc
  {
    uint8_t           bmRequestType;
    uint8_t           bRequest;
    uint16_t_uint8_t  wValue;
    uint16_t_uint8_t  wIndex;
    uint16_t_uint8_t  wLength;
  } b;
}
USB_Setup_TypeDef;

uint32_t *prog_ptr = NULL;
bool unlocked = false;

#define CAN CAN2

#define CAN_BL_INPUT 0x1
#define CAN_BL_OUTPUT 0x2

int can_control_msg(USB_Setup_TypeDef *setup, uint8_t *resp) {
  int resp_len = 0;

  // flasher machine
  memset(resp, 0, 4);
  memcpy(resp+4, "\xde\xad\xd0\x0d", 4);
  resp[0] = 0xff;
  resp[2] = setup->b.bRequest;
  resp[3] = ~setup->b.bRequest;
  *((uint32_t **)&resp[8]) = prog_ptr;
  resp_len = 0xc;

  int sec;
  switch (setup->b.bRequest) {
    // **** 0xb0: flasher echo
    case 0xb0:
      resp[1] = 0xff;
      break;
    // **** 0xb1: unlock flash
    case 0xb1:
      if (flash_is_locked()) {
        flash_unlock();
        resp[1] = 0xff;
      }
      out_enable(LED_GREEN, true);
      unlocked = true;
      prog_ptr = (uint32_t *)APP_START_ADDRESS;
      break;
    // **** 0xb2: erase sector
    case 0xb2:
      sec = setup->b.wValue.w;
      if (flash_erase_sector(sec, unlocked)) {
        resp[1] = 0xff;
      }
      break;
    // **** 0xd0: fetch serial number
    case 0xd0:
        // addresses are OTP
        if (setup->b.wValue.w == 1) {
          memcpy(resp, (void *)DEVICE_SERIAL_NUMBER_ADDRESS, 0x10);
          resp_len = 0x10;
        } else {
          get_provision_chunk(resp);
          resp_len = PROVISION_CHUNK_LEN;
        }
      break;
    // **** 0xd1: enter bootloader mode
    case 0xd1:
      switch (setup->b.wValue.w) {
        case 1:
          enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
          NVIC_SystemReset();
          break;
      }
      break;
    // **** 0xd6: get version
    case 0xd6:
      COMPILE_TIME_ASSERT(sizeof(gitversion) <= 0x40U);
      memcpy(resp, gitversion, sizeof(gitversion));
      resp_len = sizeof(gitversion);
      break;
    // **** 0xd8: reset ST
    case 0xd8:
      NVIC_SystemReset();
      break;
  }
  return resp_len;
}

void flash_data(void *data, int len) {
  out_enable(LED_RED, false);
  for (int i = 0; i < len/4; i++) {
    flash_write_word(prog_ptr, *(uint32_t*)(data+(i*4)));
    prog_ptr++;
  }
  out_enable(LED_RED, true);
}

int prep_data(uint8_t *data, uint8_t *data_out) {
  int resp_len = 0;
  switch (data[0]) {
    case 0:
      // control transfer
      resp_len = can_control_msg((USB_Setup_TypeDef *)(data+4), data_out);
      break;
    case 2:
      // ep 2, flash!
      flash_data(data+4, data[2]);
      break;
  }
  return resp_len;
}

void CAN2_TX_IRQHandler(void) {
  // clear interrupt
  CAN->TSR |= CAN_TSR_RQCP0;
}

#define ISOTP_BUF_SIZE 0x110

uint8_t isotp_buf[ISOTP_BUF_SIZE];
uint8_t *isotp_buf_ptr = NULL;
int isotp_buf_remain = 0;

uint8_t isotp_buf_out[ISOTP_BUF_SIZE];
uint8_t *isotp_buf_out_ptr = NULL;
int isotp_buf_out_remain = 0;
int isotp_buf_out_idx = 0;

void bl_can_send(uint8_t *odat) {
  // wait for send
  while (!(CAN->TSR & CAN_TSR_TME0));

  // send continue
  CAN->sTxMailBox[0].TDLR = ((uint32_t*)odat)[0];
  CAN->sTxMailBox[0].TDHR = ((uint32_t*)odat)[1];
  CAN->sTxMailBox[0].TDTR = 8;
  CAN->sTxMailBox[0].TIR = (CAN_BL_OUTPUT << 21) | 1;
}

void CAN2_RX0_IRQHandler(void) {
  while (CAN->RF0R & CAN_RF0R_FMP0) {
    if ((CAN->sFIFOMailBox[0].RIR>>21) == CAN_BL_INPUT) {
      uint8_t dat[8];
      for (int i = 0; i < 8; i++) {
        dat[i] = GET_MAILBOX_BYTE(&CAN->sFIFOMailBox[0], i);
      }
      uint8_t odat[8];
      uint8_t type = dat[0] & 0xF0;
      if (type == 0x30) {
        // continue
        while (isotp_buf_out_remain > 0) {
          // wait for send
          while (!(CAN->TSR & CAN_TSR_TME0));

          odat[0] = 0x20 | isotp_buf_out_idx;
          memcpy(odat+1, isotp_buf_out_ptr, 7);
          isotp_buf_out_remain -= 7;
          isotp_buf_out_ptr += 7;
          isotp_buf_out_idx++;

          bl_can_send(odat);
        }
      } else if (type == 0x20) {
        if (isotp_buf_remain > 0) {
          memcpy(isotp_buf_ptr, dat+1, 7);
          isotp_buf_ptr += 7;
          isotp_buf_remain -= 7;
        }
        if (isotp_buf_remain <= 0) {
          // call the function
          memset(isotp_buf_out, 0, ISOTP_BUF_SIZE);
          isotp_buf_out_remain = prep_data(isotp_buf, isotp_buf_out);
          isotp_buf_out_ptr = isotp_buf_out;
          isotp_buf_out_idx = 0;

          // send initial
          if (isotp_buf_out_remain <= 7) {
            odat[0] = isotp_buf_out_remain;
            memcpy(odat+1, isotp_buf_out_ptr, isotp_buf_out_remain);
          } else {
            odat[0] = 0x10 | (isotp_buf_out_remain>>8);
            odat[1] = isotp_buf_out_remain & 0xFF;
            memcpy(odat+2, isotp_buf_out_ptr, 6);
            isotp_buf_out_remain -= 6;
            isotp_buf_out_ptr += 6;
            isotp_buf_out_idx++;
          }

          bl_can_send(odat);
        }
      } else if (type == 0x10) {
        int len = ((dat[0]&0xF)<<8) | dat[1];

        // setup buffer
        isotp_buf_ptr = isotp_buf;
        memcpy(isotp_buf_ptr, dat+2, 6);

        if (len < (ISOTP_BUF_SIZE-0x10)) {
          isotp_buf_ptr += 6;
          isotp_buf_remain = len-6;
        }

        memset(odat, 0, 8);
        odat[0] = 0x30;
        bl_can_send(odat);
      }
    }
    // next
    CAN->RF0R |= CAN_RF0R_RFOM0;
  }
}

void CAN2_SCE_IRQHandler(void) {
  llcan_clear_send(CAN);
}


void soft_flasher_start(void) {
  HAL_NVIC_SetPriority(CAN2_TX_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
  HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  HAL_NVIC_SetPriority(CAN2_SCE_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN2_SCE_IRQn);

  enter_bootloader_mode = 0;

  out_enable(TRANSCEIVER, true);

  __HAL_RCC_CAN1_CLK_ENABLE(); // Also needed for CAN2, dumb...
  __HAL_RCC_CAN2_CLK_ENABLE();

  // init can
  llcan_set_speed(CAN, 5000, false, false);
  llcan_init(CAN);

  // green LED on for flashing
  out_enable(LED_GREEN, true);

  uint64_t cnt = 0;

  for (cnt=0;;cnt++) {
    // blink the green LED fast
    out_enable(LED_GREEN, false);
    delay(500000);
    out_enable(LED_GREEN, true);
    delay(500000);
  }
}
