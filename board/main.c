#include <stdio.h>
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

extern uint8_t     inIdx;               // input index used for dual-inputs
extern uint8_t     inIdx_prev;
extern InputStruct input1[];            // input structure
extern InputStruct input2[];            // input structure

extern int16_t speedAvg;                // Average measured speed
extern int16_t speedAvgAbs;             // Average measured speed in absolute
extern volatile uint32_t timeoutCntGen; // Timeout counter for the General timeout (PPM, PWM, Nunchuk)
extern uint8_t timeoutFlgADC;           // Timeout Flag for for ADC Protection: 0 = OK, 1 = Problem detected (line disconnected or wrong ADC data)
extern uint8_t timeoutFlgSerial;        // Timeout Flag for Rx Serial command: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)

extern volatile int pwml;               // global variable for pwm left. -1000 to 1000
extern volatile int pwmr;               // global variable for pwm right. -1000 to 1000

extern uint8_t enable;                  // global variable for motor enable

extern int16_t batVoltage;              // global variable for battery voltage

//------------------------------------------------------------------------
// Global variables set here in main.c
//------------------------------------------------------------------------
uint8_t backwardDrive;
extern volatile uint32_t buzzerTimer;
volatile uint32_t main_loop_counter;
int16_t batVoltageCalib;         // global variable for calibrated battery voltage
int16_t board_temp_deg_c;        // global variable for calibrated temperature in degrees Celsius
int16_t left_dc_curr;            // global variable for Left DC Link current
int16_t right_dc_curr;           // global variable for Right DC Link current
int16_t dc_curr;                 // global variable for Total DC Link current
int16_t cmdL;                    // global variable for Left Command
int16_t cmdR;                    // global variable for Right Command

uint8_t ignition;                // global variable for ignition on SBU2 line

//------------------------------------------------------------------------
// Local variables
//------------------------------------------------------------------------
static uint32_t    buzzerTimer_prev = 0;
static uint32_t    inactivity_timeout_counter;
static MultipleTap MultipleTapBrake;    // define multiple tap functionality for the Brake pedal

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

  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_SET);   // Activate Latch

  HAL_GPIO_WritePin(IGNITION_PORT, IGNITION_PIN, GPIO_PIN_SET); // Set ignition pin HIGH (ON)
  HAL_GPIO_WritePin(CAN_STBY_PORT, CAN_STBY_PIN, GPIO_PIN_RESET); // Enable transceiver by pulling STBY pin LOW (Normal mode)

  HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_BLUE_PORT, LED_BLUE_PIN, GPIO_PIN_SET);

  __HAL_RCC_CAN1_CLK_ENABLE(); // Also needed for CAN2, dumb...
  __HAL_RCC_CAN2_CLK_ENABLE();
  llcan_set_speed(CAN2, 5000, false, false);
  llcan_init(CAN2);

  poweronMelody();

  ignition = 1;

  int32_t board_temp_adcFixdt = adc_buffer.temp << 16;  // Fixed-point filter output initialized with current ADC converted to fixed-point
  int16_t board_temp_adcFilt  = adc_buffer.temp;

  // Loop until button is released
  while(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) { HAL_Delay(10); }

  while(1) {
    if (buzzerTimer - buzzerTimer_prev > 16*DELAY_IN_MAIN_LOOP) {   // 1 ms = 16 ticks buzzerTimer

    // readCommand();                        // Read Command: input1[inIdx].cmd, input2[inIdx].cmd
    if (!enable) {
      input1[inIdx].cmd = 0;
      input2[inIdx].cmd = 0;
    } else {
      // To test motors without CAN bus commands, REMOVE
      cmdL = 60;
      cmdR = 60;
    }
    calcAvgSpeed();

    if (ignition == 0) {
      cmdL = cmdR = 0;
      enable = 0;
    }

    // ####### MOTOR ENABLING: Only if the initial input is very small (for SAFETY) #######
    if (ignition == 1 && enable == 0 && (!rtY_Left.z_errCode && !rtY_Right.z_errCode) && (input1[inIdx].cmd > -50 && input1[inIdx].cmd < 50) && (input2[inIdx].cmd > -50 && input2[inIdx].cmd < 50)){
      beepShort(6);                     // make 2 beeps indicating the motor enable
      beepShort(4); HAL_Delay(100);
      cmdL = cmdR = 0;
      enable = 1;            // enable motors
    }

    HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, !ignition);

    #ifdef ELECTRIC_BRAKE_ENABLE
      electricBrake(speedBlend, MultipleTapBrake.b_multipleTap);  // Apply Electric Brake. Only available and makes sense for TORQUE Mode
    #endif


    cmdL = CLAMP((int)cmdL, -1000, 1000); // These should be set to INPUT_MIN and INPUT_MAX from util.c, init by Input_Lim_Init()
    cmdR = CLAMP((int)cmdR, -1000, 1000);

    pwmr = -cmdR;
    pwml = cmdL;

    // ####### CALC BOARD TEMPERATURE #######
    filtLowPass32(adc_buffer.temp, TEMP_FILT_COEF, &board_temp_adcFixdt);
    board_temp_adcFilt  = (int16_t)(board_temp_adcFixdt >> 16);  // convert fixed-point to integer
    board_temp_deg_c    = (TEMP_CAL_HIGH_DEG_C - TEMP_CAL_LOW_DEG_C) * (board_temp_adcFilt - TEMP_CAL_LOW_ADC) / (TEMP_CAL_HIGH_ADC - TEMP_CAL_LOW_ADC) + TEMP_CAL_LOW_DEG_C;

    // ####### CALC CALIBRATED BATTERY VOLTAGE #######
    batVoltageCalib = batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC;

    // ####### CALC DC LINK CURRENT ####### // Doesn't work, has no DCLink
    left_dc_curr  = -(rtU_Left.i_DCLink * 100) / A2BIT_CONV;   // Left DC Link Current * 100
    right_dc_curr = -(rtU_Right.i_DCLink * 100) / A2BIT_CONV;  // Right DC Link Current * 100
    dc_curr       = left_dc_curr + right_dc_curr;            // Total DC Link Current * 100

    if (main_loop_counter % 2 == 0) { // This runs at 100Hz (must be mod 2)
      inactivity_timeout_counter = 0; // Temporarily ignore inactivity timeout
      // Will move to separate func later
      if ((CAN2->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
        uint8_t dat[8];
        uint16_t speedL = rtY_Left.n_mot;
        uint16_t speedR = -(rtY_Right.n_mot); // Invert speed sign for the right wheel
        dat[0] = (speedL >> 8U) & 0xFFU;
        dat[1] = speedL & 0xFFU;
        dat[2] = (speedR >> 8U) & 0xFFU;
        dat[3] = speedR & 0xFFU;
        dat[4] = rtY_Left.a_elecAngle;
        dat[5] = rtY_Right.a_elecAngle;
        dat[6] = (batVoltageCalib >> 8U) & 0xFFU;
        dat[7] = batVoltageCalib & 0xFFU;

        CAN2->sTxMailBox[0].TIR = (0x201U << 21U);
        CAN2->sTxMailBox[0].TDTR = 8U;
        CAN2->sTxMailBox[0].TDLR = dat[0] | (dat[1] << 8U) | (dat[2] << 16U) | (dat[3] << 24U);
        CAN2->sTxMailBox[0].TDHR = dat[4] | (dat[5]<< 8U) | (dat[6] << 16U) | (dat[7] << 24U);
        // CAN2->sTxMailBox[0].TDLR = dat[0] | (dat[1] << 8U) | (dat[2] << 16U) | (dat[3] << 24U);
        // CAN2->sTxMailBox[0].TDHR = dat[4] | (dat[5]<< 8U) | (dat[6] << 16U) | (dat[7] << 24U);
        CAN2->sTxMailBox[0].TIR |= 0x1U;
      }
    }

    if (main_loop_counter % 200 == 0) { // Runs at 1Hz
      HAL_GPIO_WritePin(LED_BLUE_PORT, LED_BLUE_PIN, GPIO_PIN_SET);
    }

    // ####### POWEROFF BY POWER-BUTTON #######
    poweroffPressCheck();

    // ####### BEEP AND EMERGENCY POWEROFF #######
    if ((TEMP_POWEROFF_ENABLE && board_temp_deg_c >= TEMP_POWEROFF && speedAvgAbs < 20) || (batVoltage < BAT_DEAD && speedAvgAbs < 20)) {  // poweroff before mainboard burns OR low bat 3
      poweroff();
    } else if (rtY_Left.z_errCode || rtY_Right.z_errCode) {                                           // 1 beep (low pitch): Motor error, disable motors
      enable = 0;
      beepCount(1, 24, 1);
    } else if (timeoutFlgADC) {                                                                       // 2 beeps (low pitch): ADC timeout
      beepCount(2, 24, 1);
    } else if (timeoutFlgSerial) {                                                                    // 3 beeps (low pitch): Serial timeout
      beepCount(3, 24, 1);
    } else if (TEMP_WARNING_ENABLE && board_temp_deg_c >= TEMP_WARNING) {                             // 5 beeps (low pitch): Mainboard temperature warning
      beepCount(5, 24, 1);
    } else if (BAT_LVL1_ENABLE && batVoltage < BAT_LVL1) {                                            // 1 beep fast (medium pitch): Low bat 1
      beepCount(0, 10, 6);
    } else if (BAT_LVL2_ENABLE && batVoltage < BAT_LVL2) {                                            // 1 beep slow (medium pitch): Low bat 2
      beepCount(0, 10, 30);
    } else if (BEEPS_BACKWARD && ((speedAvg < 0) || MultipleTapBrake.b_multipleTap)) { // 1 beep fast (high pitch): Backward spinning motors
      beepCount(0, 5, 1);
      backwardDrive = 1;
    } else {  // do not beep
      beepCount(0, 0, 0);
      backwardDrive = 0;
    }


    // ####### INACTIVITY TIMEOUT #######
    if (ABS(cmdL) > 50 || ABS(cmdR) > 50) {
      inactivity_timeout_counter = 0;
    } else {
      inactivity_timeout_counter++;
    }
    if (inactivity_timeout_counter > (INACTIVITY_TIMEOUT * 60 * 1000) / (DELAY_IN_MAIN_LOOP + 1)) {  // rest of main loop needs maybe 1ms
      poweroff();
    }


    // HAL_GPIO_TogglePin(LED_PORT, LED_PIN);                 // This is to measure the main() loop duration with an oscilloscope connected to LED_PIN
    // Update states
    inIdx_prev = inIdx;
    buzzerTimer_prev = buzzerTimer;
    main_loop_counter++;
    }
  }
}
