#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "defines.h"
#include "config.h"
#include "util.h"
#include "bldc/BLDC_controller.h"
#include "bldc/rtwtypes.h"

/* =========================== Variable Definitions =========================== */

//------------------------------------------------------------------------
// Global variables set externally
//------------------------------------------------------------------------
extern volatile adc_buf_t adc_buffer;

extern int16_t batVoltage;
extern uint8_t backwardDrive;
extern uint8_t buzzerCount;             // global variable for the buzzer counts. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerFreq;              // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern;           // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...

extern uint8_t enable;                  // global variable for motor enable

extern volatile uint32_t timeoutCntGen; // global counter for general timeout counter
extern volatile uint32_t main_loop_counter;

extern uint8_t ignition;                // global variable for ignition on SBU2 line

//------------------------------------------------------------------------
// Global variables set here in util.c
//------------------------------------------------------------------------
// Matlab defines - from auto-code generation
//---------------
RT_MODEL rtM_Left_;                     /* Real-time model */
RT_MODEL rtM_Right_;                    /* Real-time model */
RT_MODEL *const rtM_Left  = &rtM_Left_;
RT_MODEL *const rtM_Right = &rtM_Right_;

extern P rtP_Left;                      /* Block parameters (auto storage) */
DW       rtDW_Left;                     /* Observable states */
ExtU     rtU_Left;                      /* External inputs */
ExtY     rtY_Left;                      /* External outputs */

P        rtP_Right;                     /* Block parameters (auto storage) */
DW       rtDW_Right;                    /* Observable states */
ExtU     rtU_Right;                     /* External inputs */
ExtY     rtY_Right;                     /* External outputs */
//---------------

uint8_t  inIdx      = 0;
uint8_t  inIdx_prev = 0;

InputStruct input1[1] = { {0, 0, 0, PRI_INPUT1} };
InputStruct input2[1] = { {0, 0, 0, PRI_INPUT2} };

int16_t  speedAvg;                      // average measured speed
int16_t  speedAvgAbs;                   // average measured speed in absolute
uint8_t  timeoutFlgADC    = 0;          // Timeout Flag for ADC Protection:    0 = OK, 1 = Problem detected (line disconnected or wrong ADC data)
uint8_t  timeoutFlgSerial = 0;          // Timeout Flag for Rx Serial command: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)

uint8_t  ctrlModReqRaw = CTRL_MOD_REQ;
uint8_t  ctrlModReq    = CTRL_MOD_REQ;  // Final control mode request
uint8_t nunchuk_connected = 0;

//------------------------------------------------------------------------
// Local variables
//------------------------------------------------------------------------
static int16_t INPUT_MAX;             // [-] Input target maximum limitation
static int16_t INPUT_MIN;             // [-] Input target minimum limitation
static uint8_t  cur_spd_valid  = 0;

/* =========================== Initialization Functions =========================== */

void BLDC_Init(void) {
  /* Set BLDC controller parameters */
  rtP_Left.b_angleMeasEna       = 0;            // Motor angle input: 0 = estimated angle, 1 = measured angle (e.g. if encoder is available)
  rtP_Left.z_selPhaCurMeasABC   = 0;            // Left motor measured current phases {Green, Blue} = {iA, iB} -> do NOT change
  rtP_Left.z_ctrlTypSel         = CTRL_TYP_SEL;
  rtP_Left.b_diagEna            = DIAG_ENA;
  rtP_Left.i_max                = (I_MOT_MAX * A2BIT_CONV) << 4;        // fixdt(1,16,4)
  rtP_Left.n_max                = N_MOT_MAX << 4;                       // fixdt(1,16,4)
  rtP_Left.b_fieldWeakEna       = FIELD_WEAK_ENA;
  rtP_Left.id_fieldWeakMax      = (FIELD_WEAK_MAX * A2BIT_CONV) << 4;   // fixdt(1,16,4)
  rtP_Left.a_phaAdvMax          = PHASE_ADV_MAX << 4;                   // fixdt(1,16,4)
  rtP_Left.r_fieldWeakHi        = FIELD_WEAK_HI << 4;                   // fixdt(1,16,4)
  rtP_Left.r_fieldWeakLo        = FIELD_WEAK_LO << 4;                   // fixdt(1,16,4)

  rtP_Right                     = rtP_Left;     // Copy the Left motor parameters to the Right motor parameters
  rtP_Right.z_selPhaCurMeasABC  = 1;            // Right motor measured current phases {Blue, Yellow} = {iB, iC} -> do NOT change

  /* Pack LEFT motor data into RTM */
  rtM_Left->defaultParam        = &rtP_Left;
  rtM_Left->dwork               = &rtDW_Left;
  rtM_Left->inputs              = &rtU_Left;
  rtM_Left->outputs             = &rtY_Left;

  /* Pack RIGHT motor data into RTM */
  rtM_Right->defaultParam       = &rtP_Right;
  rtM_Right->dwork              = &rtDW_Right;
  rtM_Right->inputs             = &rtU_Right;
  rtM_Right->outputs            = &rtY_Right;

  /* Initialize BLDC controllers */
  BLDC_controller_initialize(rtM_Left);
  BLDC_controller_initialize(rtM_Right);
}

/* =========================== General Functions =========================== */

void poweronMelody(void) {
    buzzerCount = 0;  // prevent interraction with beep counter
    for (int i = 8; i >= 0; i--) {
      buzzerFreq = (uint8_t)i;
      HAL_Delay(100);
    }
    buzzerFreq = 0;
}

void beepCount(uint8_t cnt, uint8_t freq, uint8_t pattern) {
    buzzerCount   = cnt;
    buzzerFreq    = freq;
    buzzerPattern = pattern;
}

void beepLong(uint8_t freq) {
    buzzerCount = 0;  // prevent interraction with beep counter
    buzzerFreq = freq;
    HAL_Delay(500);
    buzzerFreq = 0;
}

void beepShort(uint8_t freq) {
    buzzerCount = 0;  // prevent interraction with beep counter
    buzzerFreq = freq;
    HAL_Delay(100);
    buzzerFreq = 0;
}

void beepShortMany(uint8_t cnt, int8_t dir) {
    if (dir >= 0) {   // increasing tone
      for(uint8_t i = 2*cnt; i >= 2; i=i-2) {
        beepShort(i + 3);
      }
    } else {          // decreasing tone
      for(uint8_t i = 2; i <= 2*cnt; i=i+2) {
        beepShort(i + 3);
      }
    }
}

void calcAvgSpeed(void) {
    // Calculate measured average speed. The minus sign (-) is because motors spin in opposite directions
      speedAvg    = ( rtY_Left.n_mot - rtY_Right.n_mot) / 2;

    // Handle the case when SPEED_COEFFICIENT sign is negative (which is when most significant bit is 1)
    if (SPEED_COEFFICIENT & (1 << 16)) {
      speedAvg    = -speedAvg;
    }
    speedAvgAbs   = ABS(speedAvg);
}

 /*
 * Update Maximum Motor Current Limit (via ADC1) and Maximum Speed Limit (via ADC2)
 * Procedure:
 * - press the power button for more than 5 sec and immediatelly after the beep sound press one more time shortly
 * - move and hold the pots to a desired limit position for Current and Speed
 * - press the power button to confirm or wait for the 10 sec timeout
 */
void updateCurSpdLim(void) {
  calcAvgSpeed();
  if (speedAvgAbs > 5) {    // do not enter this mode if motors are spinning
    return;
  }

  int32_t  input1_fixdt = input1[inIdx].raw << 16;
  int32_t  input2_fixdt = input2[inIdx].raw << 16;
  uint16_t cur_factor;    // fixdt(0,16,16)
  uint16_t spd_factor;    // fixdt(0,16,16)
  uint16_t cur_spd_timeout = 0;
  cur_spd_valid = 0;

  // Wait for the power button press
  while (!HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) && cur_spd_timeout++ < 2000) {  // 10 sec timeout
    filtLowPass32(input1[inIdx].raw, FILTER, &input1_fixdt);
    filtLowPass32(input2[inIdx].raw, FILTER, &input2_fixdt);
    HAL_Delay(5);
  }
  // Calculate scaling factors
  cur_factor = CLAMP((input1_fixdt - (input1[inIdx].min << 16)) / (input1[inIdx].max - input1[inIdx].min), 6553, 65535);    // ADC1, MIN_cur(10%) = 1.5 A 
  spd_factor = CLAMP((input2_fixdt - (input2[inIdx].min << 16)) / (input2[inIdx].max - input2[inIdx].min), 3276, 65535);    // ADC2, MIN_spd(5%)  = 50 rpm

  if (input1[inIdx].typ != 0){
    // Update current limit
    rtP_Left.i_max = rtP_Right.i_max  = (int16_t)((I_MOT_MAX * A2BIT_CONV * cur_factor) >> 12);    // fixdt(0,16,16) to fixdt(1,16,4)
    cur_spd_valid   = 1;  // Mark update to be saved in Flash at shutdown
  }

  if (input2[inIdx].typ != 0){
    // Update speed limit
    rtP_Left.n_max = rtP_Right.n_max  = (int16_t)((N_MOT_MAX * spd_factor) >> 12);                 // fixdt(0,16,16) to fixdt(1,16,4)
    cur_spd_valid  += 2;  // Mark update to be saved in Flash at shutdown
  }
}

 /*
 * Electric Brake Function
 * In case of TORQUE mode, this function replaces the motor "freewheel" with a constant braking when the input torque request is 0.
 * This is useful when a small amount of motor braking is desired instead of "freewheel".
 *
 * Input: speedBlend = fixdt(0,16,15), reverseDir = {0, 1}
 * Output: input2.cmd (Throtle) with brake component included
 */
void electricBrake(uint16_t speedBlend, uint8_t reverseDir) {
  #if defined(ELECTRIC_BRAKE_ENABLE) && (CTRL_TYP_SEL == FOC_CTRL) && (CTRL_MOD_REQ == TRQ_MODE)
    int16_t brakeVal;

    // Make sure the Brake pedal is opposite to the direction of motion AND it goes to 0 as we reach standstill (to avoid Reverse driving) 
    if (speedAvg > 0) {
      brakeVal = (int16_t)((-ELECTRIC_BRAKE_MAX * speedBlend) >> 15);
    } else {
      brakeVal = (int16_t)(( ELECTRIC_BRAKE_MAX * speedBlend) >> 15);
    }

    // Check if direction is reversed
    if (reverseDir) {
      brakeVal = -brakeVal;
    }

    // Calculate the new input2.cmd with brake component included
    if (input2[inIdx].cmd >= 0 && input2[inIdx].cmd < ELECTRIC_BRAKE_THRES) {
      input2[inIdx].cmd = MAX(brakeVal, ((ELECTRIC_BRAKE_THRES - input2[inIdx].cmd) * brakeVal) / ELECTRIC_BRAKE_THRES);
    } else if (input2[inIdx].cmd >= -ELECTRIC_BRAKE_THRES && input2[inIdx].cmd < 0) {
      input2[inIdx].cmd = MIN(brakeVal, ((ELECTRIC_BRAKE_THRES + input2[inIdx].cmd) * brakeVal) / ELECTRIC_BRAKE_THRES);
    } else if (input2[inIdx].cmd >= ELECTRIC_BRAKE_THRES) {
      input2[inIdx].cmd = MAX(brakeVal, ((input2[inIdx].cmd - ELECTRIC_BRAKE_THRES) * INPUT_MAX) / (INPUT_MAX - ELECTRIC_BRAKE_THRES));
    } else {  // when (input2.cmd < -ELECTRIC_BRAKE_THRES)
      input2[inIdx].cmd = MIN(brakeVal, ((input2[inIdx].cmd + ELECTRIC_BRAKE_THRES) * INPUT_MIN) / (INPUT_MIN + ELECTRIC_BRAKE_THRES));
    }
  #endif
}

 /*
 * Check Input Type
 * This function identifies the input type: 0: Disabled, 1: Normal Pot, 2: Middle Resting Pot
 */
int checkInputType(int16_t min, int16_t mid, int16_t max){

  int type = 0;
  int16_t threshold = 200;

  if ((min / threshold) == (max / threshold) || (mid / threshold) == (max / threshold) || min > max || mid > max) {
    type = 0;
  } else {
    if ((min / threshold) == (mid / threshold)){
      type = 1;
    } else {
      type = 2;
    }
  }

  return type;
}

/* =========================== Input Functions =========================== */

 /*
 * Calculate Input Command
 * This function realizes dead-band around 0 and scales the input between [out_min, out_max]
 */
void calcInputCmd(InputStruct *in, int16_t out_min, int16_t out_max) {
  switch (in->typ){
    case 1: // Input is a normal pot
      in->cmd = CLAMP(MAP(in->raw, in->min, in->max, 0, out_max), 0, out_max);
      break;
    case 2: // Input is a mid resting pot
      if( in->raw > in->mid - in->dband && in->raw < in->mid + in->dband ) {
        in->cmd = 0;
      } else if(in->raw > in->mid) {
        in->cmd = CLAMP(MAP(in->raw, in->mid + in->dband, in->max, 0, out_max), 0, out_max);
      } else {
        in->cmd = CLAMP(MAP(in->raw, in->mid - in->dband, in->min, 0, out_min), out_min, 0);
      }
      break;
    default: // Input is ignored
      in->cmd = 0;
      break;
  }
}

 /*
 * Function to handle the ADC, UART and General timeout (Nunchuk, PPM, PWM)
 */
void handleTimeout(void) {
    // In case of timeout bring the system to a Safe State
    if (timeoutFlgADC || timeoutFlgSerial) {
      ctrlModReq  = OPEN_MODE;                                          // Request OPEN_MODE. This will bring the motor power to 0 in a controlled way
      input1[inIdx].cmd  = 0;
      input2[inIdx].cmd  = 0;
    } else {
      ctrlModReq  = ctrlModReqRaw;                                      // Follow the Mode request
    }

    // Beep in case of Input index change
    if (inIdx && !inIdx_prev) {                                         // rising edge
      beepShort(8);
    } else if (!inIdx && inIdx_prev) {                                  // falling edge
      beepShort(18);
    }
}

 /*
 * Function to calculate the command to the motors. This function also manages:
 * - timeout detection
 * - MIN/MAX limitations and deadband
 */
void readCommand(void) {
    calcInputCmd(&input1[inIdx], INPUT_MIN, INPUT_MAX);
    calcInputCmd(&input2[inIdx], INPUT_MIN, INPUT_MAX);

    handleTimeout();
}

/* =========================== Poweroff Functions =========================== */

void poweroff(void) {
  enable = 0;
  buzzerCount = 0;  // prevent interraction with beep counter
  buzzerPattern = 0;
  for (int i = 0; i < 8; i++) {
    buzzerFreq = (uint8_t)i;
    HAL_Delay(100);
  }
  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_RESET);
  while(1) {
    // Temporarily, to see that we went to power off but can't switch the latch
    HAL_GPIO_TogglePin(LED_RED_PORT, LED_RED_PIN);
    HAL_Delay(100);
  }
}


void poweroffPressCheck(void) {
  if(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
    uint16_t cnt_press = 0;
    while(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
      HAL_Delay(10);
      cnt_press++;
      if (cnt_press == 2 * 100) { poweroff(); }
    }
    if (cnt_press > 8) {
      HAL_GPIO_TogglePin(IGNITION_PORT, IGNITION_PIN);
      ignition = !ignition;
      beepShort(5);
    }
  }
}



/* =========================== Filtering Functions =========================== */

  /* Low pass filter fixed-point 32 bits: fixdt(1,32,16)
  * Max:  32767.99998474121
  * Min: -32768
  * Res:  1.52587890625e-05
  *
  * Inputs:       u     = int16 or int32
  * Outputs:      y     = fixdt(1,32,16)
  * Parameters:   coef  = fixdt(0,16,16) = [0,65535U]
  *
  * Example:
  * If coef = 0.8 (in floating point), then coef = 0.8 * 2^16 = 52429 (in fixed-point)
  * filtLowPass16(u, 52429, &y);
  * yint = (int16_t)(y >> 16); // the integer output is the fixed-point ouput shifted by 16 bits
  */
void filtLowPass32(int32_t u, uint16_t coef, int32_t *y) {
  int64_t tmp;
  tmp = ((int64_t)((u << 4) - (*y >> 12)) * coef) >> 4;
  tmp = CLAMP(tmp, -2147483648LL, 2147483647LL);  // Overflow protection: 2147483647LL = 2^31 - 1
  *y = (int32_t)tmp + (*y);
}
  // Old filter
  // Inputs:       u     = int16
  // Outputs:      y     = fixdt(1,32,20)
  // Parameters:   coef  = fixdt(0,16,16) = [0,65535U]
  // yint = (int16_t)(y >> 20); // the integer output is the fixed-point ouput shifted by 20 bits
  // void filtLowPass32(int16_t u, uint16_t coef, int32_t *y) {
  //   int32_t tmp;
  //   tmp = (int16_t)(u << 4) - (*y >> 16);
  //   tmp = CLAMP(tmp, -32768, 32767);  // Overflow protection
  //   *y  = coef * tmp + (*y);
  // }


  /* rateLimiter16(int16_t u, int16_t rate, int16_t *y);
  * Inputs:       u     = int16
  * Outputs:      y     = fixdt(1,16,4)
  * Parameters:   rate  = fixdt(1,16,4) = [0, 32767] Do NOT make rate negative (>32767)
  */
void rateLimiter16(int16_t u, int16_t rate, int16_t *y) {
  int16_t q0;
  int16_t q1;

  q0 = (u << 4)  - *y;

  if (q0 > rate) {
    q0 = rate;
  } else {
    q1 = -rate;
    if (q0 < q1) {
      q0 = q1;
    }
  }

  *y = q0 + *y;
}


  /* mixerFcn(rtu_speed, rtu_steer, &rty_speedR, &rty_speedL);
  * Inputs:       rtu_speed, rtu_steer                  = fixdt(1,16,4)
  * Outputs:      rty_speedR, rty_speedL                = int16_t
  * Parameters:   SPEED_COEFFICIENT, STEER_COEFFICIENT  = fixdt(0,16,14)
  */
void mixerFcn(int16_t rtu_speed, int16_t rtu_steer, int16_t *rty_speedR, int16_t *rty_speedL) {
  int16_t prodSpeed;
  int16_t prodSteer;
  int32_t tmp;

  prodSpeed   = (int16_t)((rtu_speed * (int16_t)SPEED_COEFFICIENT) >> 14);
  prodSteer   = (int16_t)((rtu_steer * (int16_t)STEER_COEFFICIENT) >> 14);

  tmp         = prodSpeed - prodSteer;  
  tmp         = CLAMP(tmp, -32768, 32767);  // Overflow protection
  *rty_speedR = (int16_t)(tmp >> 4);        // Convert from fixed-point to int
  *rty_speedR = CLAMP(*rty_speedR, INPUT_MIN, INPUT_MAX);

  tmp         = prodSpeed + prodSteer;
  tmp         = CLAMP(tmp, -32768, 32767);  // Overflow protection
  *rty_speedL = (int16_t)(tmp >> 4);        // Convert from fixed-point to int
  *rty_speedL = CLAMP(*rty_speedL, INPUT_MIN, INPUT_MAX);
}



/* =========================== Multiple Tap Function =========================== */

  /* multipleTapDet(int16_t u, uint32_t timeNow, MultipleTap *x)
  * This function detects multiple tap presses, such as double tapping, triple tapping, etc.
  * Inputs:       u = int16_t (input signal); timeNow = uint32_t (current time)
  * Outputs:      x->b_multipleTap (get the output here)
  */
void multipleTapDet(int16_t u, uint32_t timeNow, MultipleTap *x) {
  uint8_t 	b_timeout;
  uint8_t 	b_hyst;
  uint8_t 	b_pulse;
  uint8_t 	z_pulseCnt;
  uint8_t   z_pulseCntRst;
  uint32_t 	t_time; 

  // Detect hysteresis
  if (x->b_hysteresis) {
    b_hyst = (u > MULTIPLE_TAP_LO);
  } else {
    b_hyst = (u > MULTIPLE_TAP_HI);
  }

  // Detect pulse
  b_pulse = (b_hyst != x->b_hysteresis);

  // Save time when first pulse is detected
  if (b_hyst && b_pulse && (x->z_pulseCntPrev == 0)) {
    t_time = timeNow;
  } else {
    t_time = x->t_timePrev;
  }

  // Create timeout boolean
  b_timeout = (timeNow - t_time > MULTIPLE_TAP_TIMEOUT);

  // Create pulse counter
  if ((!b_hyst) && (x->z_pulseCntPrev == 0)) {
    z_pulseCnt = 0U;
  } else {
    z_pulseCnt = b_pulse;
  }

  // Reset counter if we detected complete tap presses OR there is a timeout
  if ((x->z_pulseCntPrev >= MULTIPLE_TAP_NR) || b_timeout) {
    z_pulseCntRst = 0U;
  } else {
    z_pulseCntRst = x->z_pulseCntPrev;
  }
  z_pulseCnt = z_pulseCnt + z_pulseCntRst;

  // Check if complete tap presses are detected AND no timeout
  if ((z_pulseCnt >= MULTIPLE_TAP_NR) && (!b_timeout)) {
    x->b_multipleTap = !x->b_multipleTap;	// Toggle output
  }

  // Update states
  x->z_pulseCntPrev = z_pulseCnt;
  x->b_hysteresis 	= b_hyst;
  x->t_timePrev 	  = t_time;
}

uint8_t crc_checksum(uint8_t *dat, int len, const uint8_t poly) {
  uint8_t crc = 0xFFU;
  int i;
  int j;
  for (i = len - 1; i >= 0; i--) {
    crc ^= dat[i];
    for (j = 0; j < 8; j++) {
      if ((crc & 0x80U) != 0U) {
        crc = (uint8_t)((crc << 1) ^ poly);
      }
      else {
        crc <<= 1;
      }
    }
  }
  return crc;
}
