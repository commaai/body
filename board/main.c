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
int16_t batVoltageCalib;         // global variable for calibrated battery voltage
int16_t board_temp_deg_c;        // global variable for calibrated temperature in degrees Celsius
int16_t cmdL;                    // global variable for Left Command
int16_t cmdR;                    // global variable for Right Command

uint8_t ignition;                // global variable for ignition on SBU2 line
bool forward = true;

//------------------------------------------------------------------------
// Local variables
//------------------------------------------------------------------------
static uint32_t buzzerTimer_prev = 0;

const uint8_t crc_poly = 0xD5U;  // standard crc8


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
  out_enable(IGNITION, true);
  out_enable(TRANSCEIVER, true);

  // Reset LEDs upon startup
  out_enable(LED_RED, false);
  out_enable(LED_GREEN, false);
  out_enable(LED_BLUE, false);

  __HAL_RCC_CAN1_CLK_ENABLE(); // Also needed for CAN2, dumb...
  __HAL_RCC_CAN2_CLK_ENABLE();
  llcan_set_speed(CAN2, 5000, false, false);
  llcan_init(CAN2);

  out_enable(LED_GREEN, true);
  poweronMelody();
  out_enable(LED_GREEN, false);
  poweronMelody();
  out_enable(LED_GREEN, true);
  poweronMelody();
  out_enable(LED_GREEN, false);

  ignition = 1;

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
      if (!enable_motors) {
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

      if (cmdL >= 1000) {
        forward = false;
      } else if (cmdL <= -1000) {
        forward = true;
      }

      if (main_loop_counter % 20 == 0) { // 10Hz
        if (forward) {
          out_enable(LED_BLUE, true);
          out_enable(LED_RED, false);
          cmdL += 100;
          cmdR += 100;
        } else {
          out_enable(LED_RED, true);
          out_enable(LED_BLUE, false);
          cmdL -= 100;
          cmdR -= 100;
        }
      }

      pwml = CLAMP((int)cmdL, -1000, 1000);
      pwmr = -CLAMP((int)cmdR, -1000, 1000);

      // ####### CALC BOARD TEMPERATURE #######
      filtLowPass32(adc_buffer.temp, TEMP_FILT_COEF, &board_temp_adcFixdt);
      board_temp_adcFilt  = (int16_t)(board_temp_adcFixdt >> 16);  // convert fixed-point to integer
      board_temp_deg_c    = (TEMP_CAL_HIGH_DEG_C - TEMP_CAL_LOW_DEG_C) * (board_temp_adcFilt - TEMP_CAL_LOW_ADC) / (TEMP_CAL_HIGH_ADC - TEMP_CAL_LOW_ADC) + TEMP_CAL_LOW_DEG_C;

      // ####### CALC CALIBRATED BATTERY VOLTAGE #######
      batVoltageCalib = batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC

      if (main_loop_counter % 200 == 0) { // Runs at ~1Hz
        uint8_t dat[4];
        dat[0] = (board_temp_deg_c >> 8U) & 0xFFU;
        dat[1] = board_temp_deg_c & 0xFFU;
        dat[2] = (batVoltageCalib >> 8U) & 0xFFU;
        dat[3] = batVoltageCalib & 0xFFU;

        // MCU temp(2), battery voltage(2)
        can_send_msg(0x203U, 0x0U, ((dat[3] << 24U) | (dat[2] << 16U) | (dat[1] << 8U) | dat[0]), 4U);
      }

      poweroffPressCheck();

      if ((TEMP_POWEROFF_ENABLE && board_temp_deg_c >= TEMP_POWEROFF && speedAvgAbs < 20) || (batVoltage < BAT_DEAD && speedAvgAbs < 20)) {  // poweroff before mainboard burns OR low bat 3
        poweroff();
      } else if (rtY_Left.z_errCode || rtY_Right.z_errCode) {                                           // 1 beep (low pitch): Motor error, disable motors
        enable_motors = 0;
        beepCount(1, 24, 1);
      } else if (TEMP_WARNING_ENABLE && board_temp_deg_c >= TEMP_WARNING) {                             // 5 beeps (low pitch): Mainboard temperature warning
        beepCount(5, 24, 1);
      } else if (BAT_LVL1_ENABLE && batVoltage < BAT_LVL1) {                                            // 1 beep fast (medium pitch): Low bat 1
        beepCount(0, 10, 6);
      } else if (BAT_LVL2_ENABLE && batVoltage < BAT_LVL2) {                                            // 1 beep slow (medium pitch): Low bat 2
        beepCount(0, 10, 30);
      } else {  // do not beep
        beepCount(0, 0, 0);
      }

      buzzerTimer_prev = buzzerTimer;
      main_loop_counter = (main_loop_counter < MAX_uint32_T) ? (main_loop_counter+1) : 0;
      torque_cmd_timeout = (torque_cmd_timeout < MAX_uint32_T) ? (torque_cmd_timeout+1) : 0;
    }
  }
}
