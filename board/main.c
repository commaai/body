#include <stdint.h>
#include <stdbool.h>
#include "libc.h"
#include "stm32f4xx_hal.h"
#include "defines.h"
#include "config.h"
#include "setup.h"
#include "util.h"
#include "bldc/BLDC_controller.h"      /* BLDC's header file */
#include "bldc/rtwtypes.h"
#include "version.h"
#include "obj/gitversion.h"
#include "comms.h"
#include "drivers/clock.h"
#include "early_init.h"


uint32_t enter_bootloader_mode;

void __initialize_hardware_early(void) {
  early_initialization();
}
//------------------------------------------------------------------------
// Global variables set externally
//------------------------------------------------------------------------
extern TIM_HandleTypeDef htim_left;
extern TIM_HandleTypeDef htim_right;
extern ADC_HandleTypeDef hadc;
extern volatile adc_buf_t adc_buffer;

// Matlab defines - from auto-code generation
//---------------
extern P    rtP_Left;                   /* Block parameters (auto storage) */
extern P    rtP_Right;                  /* Block parameters (auto storage) */
extern ExtY rtY_Left;                   /* External outputs */
extern ExtY rtY_Right;                  /* External outputs */
extern ExtU rtU_Left;                   /* External inputs */
extern ExtU rtU_Right;                  /* External inputs */
//---------------

extern int16_t speedAvg;                // Average measured speed
extern int16_t speedAvgAbs;             // Average measured speed in absolute

extern volatile int pwml;               // global variable for pwm left. -1000 to 1000
extern volatile int pwmr;               // global variable for pwm right. -1000 to 1000

extern uint8_t enable_motors;                  // global variable for motor enable

extern int16_t batVoltage;              // global variable for battery voltage

//------------------------------------------------------------------------
// Global variables set here in main.c
//------------------------------------------------------------------------
extern volatile uint32_t buzzerTimer;
volatile uint32_t main_loop_counter;
volatile uint32_t torque_cmd_timeout;
volatile uint32_t ignition_off_counter;
int16_t batVoltageCalib;         // global variable for calibrated battery voltage
int16_t board_temp_deg_c;        // global variable for calibrated temperature in degrees Celsius
int16_t cmdL;                    // global variable for Left Command
int16_t cmdR;                    // global variable for Right Command

uint8_t ignition = 0;            // global variable for ignition on SBU2 line
uint8_t charger_connected = 0;   // status of the charger port
uint8_t fault_status = 0;        // fault status of the whole system
uint8_t pkt_idx = 0;             // For CAN msg counter

//------------------------------------------------------------------------
// Local variables
//------------------------------------------------------------------------
static uint32_t buzzerTimer_prev = 0U;


int main(void) {
  HAL_Init();
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  SystemClock_Config();

  __HAL_RCC_DMA2_CLK_DISABLE();
  MX_GPIO_Init();
  MX_TIM_Init();
  MX_ADC_Init();
  BLDC_Init();

  HAL_ADC_Start(&hadc);

  out_enable(POWERSWITCH, true);
  out_enable(IGNITION, ignition);
  out_enable(TRANSCEIVER, true);

  // Reset LEDs upon startup
  out_enable(LED_RED, false);
  out_enable(LED_GREEN, false);
  out_enable(LED_BLUE, false);

  __HAL_RCC_CAN1_CLK_ENABLE(); // Also needed for CAN2, dumb...
  __HAL_RCC_CAN2_CLK_ENABLE();
  llcan_set_speed(CAN2, 5000, false, false);
  llcan_init(CAN2);

  poweronMelody();

  int32_t board_temp_adcFixdt = adc_buffer.temp << 16;  // Fixed-point filter output initialized with current ADC converted to fixed-point
  int16_t board_temp_adcFilt  = adc_buffer.temp;

  // Loop until button is released
  while(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) { HAL_Delay(10); }

  while(1) {
    if (buzzerTimer - buzzerTimer_prev > 16*DELAY_IN_MAIN_LOOP) {   // 1 ms = 16 ticks buzzerTimer
      calcAvgSpeed();

      if (ignition == 0) {
        cmdL = cmdR = 0;
        enable_motors = 0;
      }
      if (!enable_motors || (torque_cmd_timeout > 20)) {
        cmdL = 0;
        cmdR = 0;
      }

      if (ignition == 1 && enable_motors == 0 && (!rtY_Left.z_errCode && !rtY_Right.z_errCode) && (ABS(cmdL) < 50 && ABS(cmdR) < 50)) {
        beepShort(6); // make 2 beeps indicating the motor enable
        beepShort(4);
        HAL_Delay(100);
        cmdL = cmdR = 0;
        enable_motors = 1; // enable motors
      }

      pwml = CLAMP((int)cmdL, -1000, 1000);
      pwmr = -CLAMP((int)cmdR, -1000, 1000);

      // ####### CALC BOARD TEMPERATURE #######
      filtLowPass32(adc_buffer.temp, TEMP_FILT_COEF, &board_temp_adcFixdt);
      board_temp_adcFilt  = (int16_t)(board_temp_adcFixdt >> 16);  // convert fixed-point to integer
      board_temp_deg_c    = (TEMP_CAL_HIGH_DEG_C - TEMP_CAL_LOW_DEG_C) * (board_temp_adcFilt - TEMP_CAL_LOW_ADC) / (TEMP_CAL_HIGH_ADC - TEMP_CAL_LOW_ADC) + TEMP_CAL_LOW_DEG_C;

      // ####### CALC CALIBRATED BATTERY VOLTAGE #######
      batVoltageCalib = batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC;

      // runs at ~100Hz
      if (main_loop_counter % 2 == 0) {
        if (ignition_off_counter <= 10) {
          // speed_L(2), speed_R(2), hall_angle_L(1), hall_angle_R(1), counter(1), checksum(1)
          uint8_t dat[8];
          uint16_t speedL = rtY_Left.n_mot;
          uint16_t speedR = -(rtY_Right.n_mot); // Invert speed sign for the right wheel
          dat[0] = (speedL >> 8U) & 0xFFU;
          dat[1] = speedL & 0xFFU;
          dat[2] = (speedR >> 8U) & 0xFFU;
          dat[3] = speedR & 0xFFU;
          dat[4] = rtY_Left.a_elecAngle;
          dat[5] = rtY_Right.a_elecAngle;
          dat[6] = pkt_idx;
          dat[7] = crc_checksum(dat, 7, crc_poly);
          can_send_msg(0x201U, ((dat[7] << 24U) | (dat[6] << 16U) | (dat[5]<< 8U) | dat[4]), ((dat[3] << 24U) | (dat[2] << 16U) | (dat[1] << 8U) | dat[0]), 8U);
          ++pkt_idx;
          pkt_idx &= 0xFU;

          // left_pha_ab(2), left_pha_bc(2), right_pha_ab(2), right_pha_bc(2)
          dat[0] = (rtU_Left.i_phaAB >> 8U) & 0xFFU;
          dat[1] = rtU_Left.i_phaAB & 0xFFU;
          dat[2] = (rtU_Left.i_phaBC >> 8U) & 0xFFU;
          dat[3] = rtU_Left.i_phaBC & 0xFFU;
          dat[4] = (rtU_Right.i_phaAB >> 8U) & 0xFFU;
          dat[5] = rtU_Right.i_phaAB & 0xFFU;
          dat[6] = (rtU_Right.i_phaBC >> 8U) & 0xFFU;
          dat[7] = rtU_Right.i_phaBC & 0xFFU;
          can_send_msg(0x204U, ((dat[7] << 24U) | (dat[6] << 16U) | (dat[5] << 8U) | dat[4]), ((dat[3] << 24U) | (dat[2] << 16U) | (dat[1] << 8U) | dat[0]), 8U);
        }
      }

      // runs at ~10Hz
      if (main_loop_counter % 20 == 0) {
        if (ignition_off_counter <= 10) {
          // fault_status(0:6), enable_motors(0:1), ignition(0:1), left motor error(1), right motor error(1), global fault status(1)
          uint8_t dat[2];
          dat[0] = (((fault_status & 0x3F) << 2U) | (enable_motors << 1U) | ignition);
          dat[1] = rtY_Left.z_errCode;
          dat[2] = rtY_Right.z_errCode;
          can_send_msg(0x202U, 0x0U, ((dat[2] << 16U) | (dat[1] << 8U) | dat[0]), 3U);
        }
        out_enable(LED_GREEN, ignition);
      }

      // runs at ~1Hz
      if (main_loop_counter % 200 == 0) {
        charger_connected = !HAL_GPIO_ReadPin(CHARGER_PORT, CHARGER_PIN);
        uint8_t battery_percent = 100 - (((420 * BAT_CELLS) - batVoltageCalib) / BAT_CELLS / VOLTS_PER_PERCENT / 100); // Battery % left

        // MCU temp(2), battery voltage(2), battery_percent(0:7), charger_connected(0:1)
        uint8_t dat[4];
        dat[0] = board_temp_deg_c & 0xFFU;
        dat[1] = (batVoltageCalib >> 8U) & 0xFFU;
        dat[2] = batVoltageCalib & 0xFFU;
        dat[3] = (((battery_percent & 0x7FU) << 1U) | charger_connected);
        can_send_msg(0x203U, 0x0U, ((dat[3] << 24U) | (dat[2] << 16U) | (dat[1] << 8U) | dat[0]), 4U);

        out_enable(LED_BLUE, false); // Reset LED after CAN RX
        out_enable(LED_GREEN, true); // Always use LED to show that body is on

        if (ignition) {
          ignition_off_counter = 0;
        } else {
          ignition_off_counter = (ignition_off_counter < MAX_uint32_T) ? (ignition_off_counter+1) : 0;
        }
      }

      process_can();
      poweroffPressCheck();

      if ((TEMP_POWEROFF_ENABLE && board_temp_deg_c >= TEMP_POWEROFF && speedAvgAbs < 20) || (batVoltage < BAT_DEAD && speedAvgAbs < 20)) {  // poweroff before mainboard burns OR low bat 3
        poweroff();
      } else if (rtY_Left.z_errCode || rtY_Right.z_errCode) { // 1 beep (low pitch): Motor error, disable motors
        enable_motors = 0;
        beepCount(1, 24, 1);
      } else if (TEMP_WARNING_ENABLE && board_temp_deg_c >= TEMP_WARNING) { // 5 beeps (low pitch): Mainboard temperature warning
        beepCount(5, 24, 1);
      } else if (batVoltage < BAT_LVL1) { // 1 beep fast (medium pitch): Low bat 1
        beepCount(0, 10, 6);
        out_enable(LED_RED, true);
      } else if (batVoltage < BAT_LVL2) { // 1 beep slow (medium pitch): Low bat 2
        beepCount(0, 10, 30);
      } else {  // do not beep
        beepCount(0, 0, 0);
        out_enable(LED_RED, false);
      }

      buzzerTimer_prev = buzzerTimer;

      main_loop_counter = (main_loop_counter < MAX_uint32_T) ? (main_loop_counter+1) : 0;
      torque_cmd_timeout = (torque_cmd_timeout < MAX_uint32_T) ? (torque_cmd_timeout+1) : 0;
    }
  }
}
