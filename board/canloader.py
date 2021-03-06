#!/usr/bin/env python3
import time
import argparse
import _thread
from panda import Panda
from panda.tests.pedal.canhandle import CanHandle

def heartbeat_thread(p):
  while True:
    try:
      p.send_heartbeat()
      time.sleep(0.5)
    except Exception:
      continue


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='Flash body over can')
  parser.add_argument("fn", type=str, nargs='?', help="flash file")
  args = parser.parse_args()

  p = Panda()
  _thread.start_new_thread(heartbeat_thread, (p,))
  p.set_safety_mode(Panda.SAFETY_BODY)

  while 1:
    if len(p.can_recv()) == 0:
      break

  p.can_send(0x250, b"\xce\xfa\xad\xde\x1e\x0b\xb0\x0a", 0)

  if args.fn:
    time.sleep(0.1)
    print("flashing", args.fn)
    code = open(args.fn, "rb").read()
    Panda.flash_static(CanHandle(p, 0), code)

  print("can flash done")
