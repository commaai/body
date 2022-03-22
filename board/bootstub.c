#define BOOTSTUB

#define VERS_TAG 0x53524556
#define MIN_VERSION 2

// ********************* Includes *********************
#include <stdbool.h>
#include <stdint.h>
#include "libc.h"
#include "stm32f4xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "drivers/clock.h"
#include "early_init.h"

#include "crypto/rsa.h"
#include "crypto/sha.h"

#include "obj/cert.h"
#include "obj/gitversion.h"

#include "drivers/llbxcan.h"
#include "drivers/llflash.h"
#include "provision.h"
#include "util.h"

#include "flasher.h"

void __initialize_hardware_early(void) {
  early_initialization();
}

void fail(void) {
  soft_flasher_start();
}

// know where to sig check
extern void *_app_start[];

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
  MX_GPIO_Init();

  out_enable(POWERSWITCH, true);
  out_enable(LED_RED, false);
  out_enable(LED_GREEN, false);
  out_enable(LED_BLUE, false);

  if (enter_bootloader_mode == ENTER_SOFTLOADER_MAGIC) {
    enter_bootloader_mode = 0;
    soft_flasher_start();
  }

  // validate length
  int len = (int)_app_start[0];
  if ((len < 8) || (len > (0x1000000 - 0x4000 - 4 - RSANUMBYTES))) goto fail;

  // compute SHA hash
  uint8_t digest[SHA_DIGEST_SIZE];
  SHA_hash(&_app_start[1], len-4, digest);

  // verify version, last bytes in the signed area
  uint32_t vers[2] = {0};
  memcpy(&vers, ((void*)&_app_start[0]) + len - sizeof(vers), sizeof(vers));
  if (vers[0] != VERS_TAG || vers[1] < MIN_VERSION) {
    goto fail;
  }

  // verify RSA signature
  if (RSA_verify(&release_rsa_key, ((void*)&_app_start[0]) + len, RSANUMBYTES, digest, SHA_DIGEST_SIZE)) {
    goto good;
  }

  // allow debug if built from source
#ifdef ALLOW_DEBUG
  if (RSA_verify(&debug_rsa_key, ((void*)&_app_start[0]) + len, RSANUMBYTES, digest, SHA_DIGEST_SIZE)) {
    goto good;
  }
#endif

// here is a failure
fail:
  fail();
  return 0;
good:
  // jump to flash
  ((void(*)(void)) _app_start[1])();
  return 0;
}
