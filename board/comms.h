// Define to prevent recursive inclusion
#ifndef COMMS_H
#define COMMS_H

#include "drivers/llbxcan.h"

extern int16_t cmdL;                    // global variable for Left Command
extern int16_t cmdR;                    // global variable for Right Command

extern uint32_t enter_bootloader_mode;

void CAN2_TX_IRQHandler(void) {
  // clear interrupt
  CAN2->TSR |= CAN_TSR_RQCP0;
}

void CAN2_SCE_IRQHandler(void) {
  llcan_clear_send(CAN2);
}

void CAN2_RX0_IRQHandler(void) {
  while ((CAN2->RF0R & CAN_RF0R_FMP0) != 0) {
    int address = CAN2->sFIFOMailBox[0].RIR >> 21;
    if (address == 0x200U) {
      if (GET_MAILBOX_BYTES_04(&CAN2->sFIFOMailBox[0]) == 0xdeadface) {
        if (GET_MAILBOX_BYTES_48(&CAN2->sFIFOMailBox[0]) == 0x0ab00b1e) {
          enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
          NVIC_SystemReset();
        }
      }

      // normal packet
      uint8_t dat[8];
      for (int i=0; i<8; i++) {
        dat[i] = GET_MAILBOX_BYTE(&CAN2->sFIFOMailBox[0], i);
      }
      cmdL = ((dat[0] << 8U) | dat[1]);
      cmdR = ((dat[2] << 8U) | dat[3]);
    }

    HAL_GPIO_WritePin(LED_BLUE_PORT, LED_BLUE_PIN, GPIO_PIN_RESET);
    // next
    CAN2->RF0R |= CAN_RF0R_RFOM0;
  }
}

#endif
