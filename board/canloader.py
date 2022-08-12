#!/usr/bin/env python3
import time
import argparse
import _thread
from panda import Panda, MCU_TYPE_F4  # pylint: disable=import-error
from panda.tests.pedal.canhandle import CanHandle  # pylint: disable=import-error


def heartbeat_thread(p):
  while True:
    try:
      p.send_heartbeat()
      time.sleep(0.5)
    except Exception:
      continue

def flasher(p, addr, file):
  p.can_send(addr, b"\xce\xfa\xad\xde\x1e\x0b\xb0\x0a", 0)
  time.sleep(0.1)
  print("flashing", file)
  code = open(file, "rb").read()
  retries = 3 # How many times to retry on timeout error
  while(retries+1>0):
    try:
      Panda.flash_static(CanHandle(p, 0), code, MCU_TYPE_F4)
    except TimeoutError:
      print("Timeout, trying again...")
      retries -= 1
    else:
      print("Successfully flashed")
      break

def flush_panda():
  while(1):
    if len(p.can_recv()) == 0:
      break


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='Flash body over can')
  parser.add_argument("fn", type=str, nargs='?', help="flash file")
  args = parser.parse_args()

  p = Panda()
  _thread.start_new_thread(heartbeat_thread, (p,))
  p.set_safety_mode(Panda.SAFETY_BODY)

  is_base_found = False
  is_knee_found = False

  print("Looking for boards to flash...")
  start_time = time.time()
  received_msgs = []
  while (time.time() - start_time) < 1:
    buff = p.can_recv()
    if len(buff):
      received_msgs.extend(buff)

  for msg in received_msgs:
    if msg[0] == 0x203:
      is_base_found = True
    elif msg[0] == 0x303:
      is_knee_found = True

  if is_knee_found and args.fn:
    print("Flashing knee motherboard")
    flasher(p, 0x350, args.fn)

  time.sleep(0.2)
  flush_panda()

  if is_base_found and args.fn:
    print("Flashing base motherboard")
    flasher(p, 0x250, args.fn)

  print("CAN flashing done")
