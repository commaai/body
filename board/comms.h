// Define to prevent recursive inclusion
#ifndef COMMS_H
#define COMMS_H

#define OFFSET            0x8U
#define BROADCAST_ADDR    0x7DFU
#define FALLBACK_ADDR     0x7E0U
#define FALLBACK_R_ADDR   (FALLBACK_ADDR + OFFSET)
#define ECU_ADDR          0x720U
#define ECU_R_ADDR        (ECU_ADDR + OFFSET)

#include "drivers/llbxcan.h"

extern int16_t cmdL;                    // global variable for Left Command
extern int16_t cmdR;                    // global variable for Right Command

extern uint32_t enter_bootloader_mode;

uint8_t uid[10];
uint32_t uds_request = 0;

void can_send_msg(uint32_t addr, uint32_t dhr, uint32_t dlr, uint8_t len) {
  // wait for send
  while (!(CAN2->TSR & CAN_TSR_TME0));

  CAN2->sTxMailBox[0].TDLR = dlr;
  CAN2->sTxMailBox[0].TDHR = dhr;
  CAN2->sTxMailBox[0].TDTR = len;
  CAN2->sTxMailBox[0].TIR = ((addr >= 0x800U) ? ((addr << 3) | (1U << 2)) : (addr << 21)) | 1;
}

void process_ubs(uint32_t addr, uint32_t dlr) {
  memcpy(uid, (void *)0x1FFF7A10U, 0xAU);

  if ((addr == BROADCAST_ADDR) || (addr == FALLBACK_ADDR)) { // OBD2 broadcast request, redirect to UDS?
    switch(dlr) {
      // VIN 09 OBD2
      case 0x020902U:
        can_send_msg(FALLBACK_R_ADDR, 0x4D4F4301U, 0x02491410U, 8U);
        uds_request = 0xF190U;
        break;
      // VIN : F190 on broadcast
      case 0x90F12203U:
        can_send_msg(FALLBACK_R_ADDR, 0x4D4F4390U, 0xF1621410U, 8U);
        break;
      // VIN continue
      case 0x30U:
        can_send_msg(FALLBACK_R_ADDR, 0x5659444FU, 0x42414D21U, 8U);
        can_send_msg(FALLBACK_R_ADDR, 0x314E4F49U, 0x53524522U, 8U);
        break;
    }
  } else if (addr == ECU_ADDR) { // UDS request
    switch(dlr) {
      // TESTER PRESENT
      case 0x3E02U:
        can_send_msg(ECU_R_ADDR, 0x0U, 0x7E02U, 8U);
        break;
      // DIAGNOSTIC SESSION CONTROL: DEFAULT
      case 0x011002U:
        can_send_msg(ECU_R_ADDR, 0x0U, 0x015002U, 8U);
        break;
      // DIAGNOSTIC SESSION CONTROL: EXTENDED
      case 0x031002U:
        can_send_msg(ECU_R_ADDR, 0x0U, 0x035002U, 8U);
        break;
      // ECU MANUFACTURING DATE : F18B
      case 0x8BF12203U:
        can_send_msg(ECU_R_ADDR, 0x2F32308BU, 0xF1620D10U, 8U);
        uds_request = 0xF18BU;
        break;
      // ECU SERIAL NUMBER : F18C
      case 0x8CF12203U:
        can_send_msg(ECU_R_ADDR, ((uid[2] << 24U) | (uid[1] << 16U) | (uid[0] << 8U) | 0x8CU), 0xF1620D10U, 8U);
        uds_request = 0xF18CU;
        break;
      // VIN : F190
      case 0x90F12203U:
        can_send_msg(ECU_R_ADDR, 0x4D4F4390U, 0xF1621410U, 8U);
        uds_request = 0xF190U;
        break;
      // SYSTEM NAME OR ENGINE TYPE : F197
      case 0x97F12203U:
        can_send_msg(ECU_R_ADDR, 0x454C4597U, 0xF1620B10U, 8U);
        uds_request = 0xF197U;
        break;

      // FLOW CONTROL MESSAGE
      case 0x30U:
        switch(uds_request) {
          // ECU MANUFACTURING DATE : F18B
          case 0xF18B:
            can_send_msg(ECU_R_ADDR, 0x32323032U, 0x2F373221U, 8U);
            uds_request = 0;
            break;
          // ECU SERIAL NUMBER : F18C
          case 0xF18C:
            can_send_msg(ECU_R_ADDR, ((uid[9] << 24U) | (uid[8] << 16U) | (uid[7]<< 8U) | uid[6]), ((uid[5] << 24U) | (uid[4] << 16U) | (uid[3] << 8U) | 0x21U), 8U);
            uds_request = 0;
            break;
          // VIN : F190
          case 0xF190U:
            can_send_msg(ECU_R_ADDR, 0x5659444FU, 0x42414D21U, 8U);
            can_send_msg(ECU_R_ADDR, 0x314E4F49U, 0x53524522U, 8U);
            uds_request = 0;
            break;
          // SYSTEM NAME OR ENGINE TYPE : F197
          case 0xF197:
            can_send_msg(ECU_R_ADDR, 0x4349U, 0x52544321U, 8U);
            uds_request = 0;
            break;
        }
        break;
    }
  }
}

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
      if ((GET_MAILBOX_BYTES_04(&CAN2->sFIFOMailBox[0]) == 0xdeadface) && (GET_MAILBOX_BYTES_48(&CAN2->sFIFOMailBox[0]) == 0x0ab00b1e)) {
        enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
        NVIC_SystemReset();
      }
      uint8_t dat[8];
      for (int i=0; i<8; i++) {
        dat[i] = GET_MAILBOX_BYTE(&CAN2->sFIFOMailBox[0], i);
      }
      cmdL = ((dat[0] << 8U) | dat[1]);
      cmdR = ((dat[2] << 8U) | dat[3]);

    } else if ((address == BROADCAST_ADDR) || (address == FALLBACK_ADDR) || (address == ECU_ADDR)) { // Process UBS and OBD2 requests
      process_ubs(address, GET_MAILBOX_BYTES_04(&CAN2->sFIFOMailBox[0]));
    }
    out_enable(LED_BLUE, true);
    // next
    CAN2->RF0R |= CAN_RF0R_RFOM0;
  }
}

#endif
