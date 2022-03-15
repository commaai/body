// Define to prevent recursive inclusion
#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f4xx_hal.h"

// ############################### DO-NOT-TOUCH SETTINGS ###############################
#define PWM_FREQ            16000     // PWM frequency in Hz / is also used for buzzer
#define DEAD_TIME              48     // PWM deadtime
#define DELAY_IN_MAIN_LOOP      5     // in ms. default 5. it is independent of all the timing critical stuff. do not touch if you do not know what you are doing.
#define TIMEOUT                20     // number of wrong / missing input commands before emergency off
#define A2BIT_CONV             50     // A to bit for current conversion on ADC. Example: 1 A = 50, 2 A = 100, etc

// This settings influences the actual sample-time. Only use definitions above
// This parameter needs to be the same as the ADC conversion for Current Phase of the FIRST Motor in setup.c
#define ADC_CONV_CLOCK_CYCLES   (ADC_SAMPLETIME_15CYCLES)

// Set the configured ADC divider. This parameter needs to be the same ADC divider as PeriphClkInit.AdcClockSelection (see main.c)
#define ADC_CLOCK_DIV           (4)

// ADC Total conversion time: this will be used to offset TIM8 in advance of TIM1 to align the Phase current ADC measurement
// This parameter is used in setup.c
#define ADC_TOTAL_CONV_TIME     (ADC_CLOCK_DIV * ADC_CONV_CLOCK_CYCLES) // = ((SystemCoreClock / ADC_CLOCK_HZ) * ADC_CONV_CLOCK_CYCLES), where ADC_CLOCK_HZ = SystemCoreClock/ADC_CLOCK_DIV
// ########################### END OF  DO-NOT-TOUCH SETTINGS ############################

// ############################### BATTERY ###############################
/* Battery voltage calibration: connect power source.
 * see How to calibrate.
 * Write debug output value nr 5 to BAT_CALIB_ADC. make and flash firmware.
 * Then you can verify voltage on debug output value 6 (to get calibrated voltage multiplied by 100).
*/
#define BAT_FILT_COEF           655       // battery voltage filter coefficient in fixed-point. coef_fixedPoint = coef_floatingPoint * 2^16. In this case 655 = 0.01 * 2^16
#define BAT_CALIB_REAL_VOLTAGE  3192      // input voltage measured by multimeter (multiplied by 100). In this case 43.00 V * 100 = 4300
#define BAT_CALIB_ADC           1275      // adc-value measured by mainboard (value nr 5 on UART debug output)
#define BAT_CELLS               7        // battery number of cells. Normal Hoverboard battery: 10s
#define BAT_LVL2_ENABLE         1         // to beep or not to beep, 1 or 0
#define BAT_LVL1_ENABLE         1         // to beep or not to beep, 1 or 0
#define BAT_BLINK_INTERVAL      80        // battery led blink interval (80 loops * 5ms ~= 400ms)
#define BAT_LVL5                (390 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE    // Green blink:  no beep
#define BAT_LVL4                (380 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE    // Yellow:       no beep
#define BAT_LVL3                (370 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE    // Yellow blink: no beep
#define BAT_LVL2                (360 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE    // Red:          gently beep at this voltage level. [V*100/cell]. In this case 3.60 V/cell
#define BAT_LVL1                (350 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE    // Red blink:    fast beep. Your battery is almost empty. Charge now! [V*100/cell]. In this case 3.50 V/cell
#define BAT_DEAD                (320 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE    // All leds off: undervoltage poweroff. (while not driving) [V*100/cell]. In this case 3.37 V/cell
// ######################## END OF BATTERY ###############################



// ############################### TEMPERATURE ###############################
/* Board overheat detection: the sensor is inside the STM/GD chip.
 * It is very inaccurate without calibration (up to 45°C). So only enable this funcion after calibration!
 * Let your board cool down.
 * see <How to calibrate.
 * Get the real temp of the chip by thermo cam or another temp-sensor taped on top of the chip and write it to TEMP_CAL_LOW_DEG_C.
 * Write debug output value 8 to TEMP_CAL_LOW_ADC. drive around to warm up the board. it should be at least 20°C warmer. repeat it for the HIGH-values.
 * Enable warning and/or poweroff and make and flash firmware.
*/
#define TEMP_FILT_COEF          655       // temperature filter coefficient in fixed-point. coef_fixedPoint = coef_floatingPoint * 2^16. In this case 655 = 0.01 * 2^16
#define TEMP_CAL_LOW_ADC        1655      // temperature 1: ADC value
#define TEMP_CAL_LOW_DEG_C      358       // temperature 1: measured temperature [°C * 10]. Here 35.8 °C
#define TEMP_CAL_HIGH_ADC       1588      // temperature 2: ADC value
#define TEMP_CAL_HIGH_DEG_C     489       // temperature 2: measured temperature [°C * 10]. Here 48.9 °C
#define TEMP_WARNING_ENABLE     0         // to beep or not to beep, 1 or 0, DO NOT ACTIVITE WITHOUT CALIBRATION!
#define TEMP_WARNING            600       // annoying fast beeps [°C * 10].  Here 60.0 °C
#define TEMP_POWEROFF_ENABLE    0         // to poweroff or not to poweroff, 1 or 0, DO NOT ACTIVITE WITHOUT CALIBRATION!
#define TEMP_POWEROFF           650       // overheat poweroff. (while not driving) [°C * 10]. Here 65.0 °C
// ######################## END OF TEMPERATURE ###############################



// ############################### MOTOR CONTROL #########################
/* GENERAL NOTES:
 * 1. The parameters here are over-writing the default motor parameters. For all the available parameters check BLDC_controller_data.c
 * 2. The parameters are represented in fixed point data type for a more efficient code execution
 * 3. For calibrating the fixed-point parameters use the Fixed-Point Viewer tool (see <https://github.com/EmanuelFeru/FixedPointViewer>)
 * 4. For more details regarding the parameters and the working principle of the controller please consult the Simulink model
 * 5. A webview was created, so Matlab/Simulink installation is not needed, unless you want to regenerate the code.
 * The webview is an html page that can be opened with browsers like: Microsoft Internet Explorer or Microsoft Edge
 *
 * NOTES Field Weakening / Phase Advance:
 * 1. The Field Weakening is a linear interpolation from 0 to FIELD_WEAK_MAX or PHASE_ADV_MAX (depeding if FOC or SIN is selected, respectively)
 * 2. The Field Weakening starts engaging at FIELD_WEAK_LO and reaches the maximum value at FIELD_WEAK_HI
 * 3. If you re-calibrate the Field Weakening please take all the safety measures! The motors can spin very fast!

   Inputs:
    - input1[inIdx].cmd and input2[inIdx].cmd: normalized input values. INPUT_MIN to INPUT_MAX
    - button1 and button2: digital input values. 0 or 1
    - adc_buffer.l_tx2 and adc_buffer.l_rx2: unfiltered ADC values (you do not need them). 0 to 4095
   Outputs:
    - cmdL and cmdR: normal driving INPUT_MIN to INPUT_MAX
*/
#define COM_CTRL        0               // [-] Commutation Control Type
#define SIN_CTRL        1               // [-] Sinusoidal Control Type
#define FOC_CTRL        2               // [-] Field Oriented Control (FOC) Type

#define OPEN_MODE       0               // [-] OPEN mode
#define VLT_MODE        1               // [-] VOLTAGE mode
#define SPD_MODE        2               // [-] SPEED mode
#define TRQ_MODE        3               // [-] TORQUE mode

// Enable/Disable Motor
//#define MOTOR_LEFT_ENA                  // [-] Enable LEFT motor.  Comment-out if this motor is not needed to be operational
#define MOTOR_RIGHT_ENA                 // [-] Enable RIGHT motor. Comment-out if this motor is not needed to be operational

// Control selections
#define CTRL_TYP_SEL    FOC_CTRL        // [-] Control type selection: COM_CTRL, SIN_CTRL, FOC_CTRL (default)
#define CTRL_MOD_REQ    TRQ_MODE        // [-] Control mode request: OPEN_MODE, VLT_MODE (default), SPD_MODE, TRQ_MODE. Note: SPD_MODE and TRQ_MODE are only available for CTRL_FOC!
#define DIAG_ENA        1               // [-] Motor Diagnostics enable flag: 0 = Disabled, 1 = Enabled (default)

// Limitation settings
#define I_MOT_MAX       15              // [A] Maximum single motor current limit
#define I_DC_MAX        17              // [A] Maximum stage2 DC Link current limit for Commutation and Sinusoidal types (This is the final current protection. Above this value, current chopping is applied. To avoid this make sure that I_DC_MAX = I_MOT_MAX + 2A)
#define N_MOT_MAX       100            // [rpm] Maximum motor speed limit // 100 ~= 52m/m

// Field Weakening / Phase Advance
#define FIELD_WEAK_ENA  0               // [-] Field Weakening / Phase Advance enable flag: 0 = Disabled (default), 1 = Enabled
#define FIELD_WEAK_MAX  5               // [A] Maximum Field Weakening D axis current (only for FOC). Higher current results in higher maximum speed. Up to 10A has been tested using 10" wheels.
#define PHASE_ADV_MAX   25              // [deg] Maximum Phase Advance angle (only for SIN). Higher angle results in higher maximum speed.
#define FIELD_WEAK_HI   1000            // (1000, 1500] Input target High threshold for reaching maximum Field Weakening / Phase Advance. Do NOT set this higher than 1500.
#define FIELD_WEAK_LO   750             // ( 500, 1000] Input target Low threshold for starting Field Weakening / Phase Advance. Do NOT set this higher than 1000.

// Extra functionality
// #define ELECTRIC_BRAKE_ENABLE           // [-] Flag to enable electric brake and replace the motor "freewheel" with a constant braking when the input torque request is 0. Only available and makes sense for TORQUE mode.
// #define ELECTRIC_BRAKE_MAX    100       // (0, 500) Maximum electric brake to be applied when input torque request is 0 (pedal fully released).
// #define ELECTRIC_BRAKE_THRES  120       // (0, 500) Threshold below at which the electric brake starts engaging.
// ########################### END OF MOTOR CONTROL ########################



// ############################## DEFAULT SETTINGS ############################
// Default settings will be applied at the end of this config file if not set before
#define INACTIVITY_TIMEOUT        8       // Minutes of not driving until poweroff. it is not very precise.
#define BEEPS_BACKWARD            0       // 0 or 1
#define ADC_MARGIN                100     // ADC input margin applied on the raw ADC min and max to make sure the MIN and MAX values are reached even in the presence of noise
#define ADC_PROTECT_TIMEOUT       100     // ADC Protection: number of wrong / missing input commands before safety state is taken
#define ADC_PROTECT_THRESH        200     // ADC Protection threshold below/above the MIN/MAX ADC values
#define AUTO_CALIBRATION_ENA              // Enable/Disable input auto-calibration by holding power button pressed. Un-comment this if auto-calibration is not needed.

/* FILTER is in fixdt(0,16,16): VAL_fixedPoint = VAL_floatingPoint * 2^16. In this case 6553 = 0.1 * 2^16
 * Value of COEFFICIENT is in fixdt(1,16,14)
 * If VAL_floatingPoint >= 0, VAL_fixedPoint = VAL_floatingPoint * 2^14
 * If VAL_floatingPoint < 0,  VAL_fixedPoint = 2^16 + floor(VAL_floatingPoint * 2^14).
*/
// Value of RATE is in fixdt(1,16,4): VAL_fixedPoint = VAL_floatingPoint * 2^4. In this case 480 = 30 * 2^4
#define DEFAULT_RATE                480   // 30.0f [-] lower value == slower rate [0, 32767] = [0.0, 2047.9375]. Do NOT make rate negative (>32767)
#define DEFAULT_FILTER              6553  // Default for FILTER 0.1f [-] lower value == softer filter [0, 65535] = [0.0 - 1.0].
#define DEFAULT_SPEED_COEFFICIENT   16384 // Default for SPEED_COEFFICIENT 1.0f [-] higher value == stronger. [0, 65535] = [-2.0 - 2.0]. In this case 16384 = 1.0 * 2^14
#define DEFAULT_STEER_COEFFICIENT   8192  // Defualt for STEER_COEFFICIENT 0.5f [-] higher value == stronger. [0, 65535] = [-2.0 - 2.0]. In this case  8192 = 0.5 * 2^14. If you do not want any steering, set it to 0.
// ######################### END OF DEFAULT SETTINGS ##########################

  #define PRI_INPUT1            3, 0, 0, 4095, 0      // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
  #define PRI_INPUT2            3, 0, 0, 4095, 0      // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
  #define FLASH_WRITE_KEY     0x1001    // Flash memory writing key. Change this key to ignore the input calibrations from the flash memory and use the ones in config.h
  #define DEBUG_SERIAL_USART3           // right sensor board cable, disable if I2C (nunchuk or lcd) is used!

// Multiple tap detection: default DOUBLE Tap on Brake pedal (4 pulses)
#define MULTIPLE_TAP_NR           2 * 2       // [-] Define tap number: MULTIPLE_TAP_NR = number_of_taps * 2, number_of_taps = 1 (for single taping), 2 (for double tapping), 3 (for triple tapping), etc...
#define MULTIPLE_TAP_HI           600         // [-] Multiple tap detection High hysteresis threshold
#define MULTIPLE_TAP_LO           200         // [-] Multiple tap detection Low hysteresis threshold
#define MULTIPLE_TAP_TIMEOUT      2000        // [ms] Multiple tap detection Timeout period. The taps need to happen within this time window to be accepted.
// ######################## END OF VARIANT_HOVERCAR SETTINGS #########################

// ############################### APPLY DEFAULT SETTINGS ###############################
#ifndef RATE
  #define RATE DEFAULT_RATE
#endif
#ifndef FILTER
  #define FILTER DEFAULT_FILTER
#endif
#ifndef SPEED_COEFFICIENT
  #define SPEED_COEFFICIENT DEFAULT_SPEED_COEFFICIENT
#endif
#ifndef STEER_COEFFICIENT
  #define STEER_COEFFICIENT DEFAULT_STEER_COEFFICIENT
#endif

// ########################### END OF APPLY DEFAULT SETTING ############################

#endif
