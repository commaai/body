#!/usr/bin/env python3
import os
import time
import argparse
import _thread
import struct

from panda import Panda  # pylint: disable=import-error
from opendbc.car.structs import CarParams
from opendbc.car.uds import CanClient, IsoTpMessage, MessageTimeoutError


REQUEST_IN = 0xC0
REQUEST_OUT = 0x40
DEFAULT_ISOTP_TIMEOUT = 2

MCU_TYPE_F4 = {
  "sector_sizes": [0x4000 for _ in range(4)] + [0x10000] + [0x20000 for _ in range(11)],
}

class CanHandle:
  def __init__(self, can_send, can_recv):
    self.client = CanClient(can_send, can_recv, tx_addr=1, rx_addr=2, bus=0)

  def transact(self, dat, timeout=DEFAULT_ISOTP_TIMEOUT, expect_disconnect=False):
    try:
      msg = IsoTpMessage(self.client, timeout=timeout)
      msg.send(dat)
      if expect_disconnect:
        deadline = time.monotonic() + timeout
        while not msg.tx_done:
          msg.recv(timeout=0)
          if not msg.tx_done and time.monotonic() > deadline:
            raise MessageTimeoutError("timeout waiting for flow control")
          time.sleep(0.01)
        return b""
      ret, _ = msg.recv()
      return ret
    except MessageTimeoutError as e:
      raise TimeoutError from e

  def controlWrite(self, request_type, request, value, index, data, timeout=DEFAULT_ISOTP_TIMEOUT, expect_disconnect=False):
    dat = struct.pack("HHBBHHH", 0, 0, request_type, request, value, index, 0)
    return self.transact(dat, timeout=timeout, expect_disconnect=expect_disconnect)

  def controlRead(self, request_type, request, value, index, length, timeout=DEFAULT_ISOTP_TIMEOUT):
    dat = struct.pack("HHBBHHH", 0, 0, request_type, request, value, index, length)
    return self.transact(dat, timeout=timeout)

  def bulkWrite(self, endpoint, data, timeout=DEFAULT_ISOTP_TIMEOUT):
    dat = struct.pack("HH", endpoint, len(data)) + data
    return self.transact(dat, timeout=timeout)

  def bulkRead(self, endpoint, timeout=DEFAULT_ISOTP_TIMEOUT):
    dat = struct.pack("HH", endpoint, 0)
    return self.transact(dat, timeout=timeout)

def heartbeat_thread(p):
  while True:
    try:
      p.send_heartbeat()
      time.sleep(0.5)
    except Exception:
      continue

def flush_panda():
  while(1):
    if len(p.can_recv()) == 0:
      break

def flasher(p, addr, file):
  p.can_send(addr, b"\xce\xfa\xad\xde\x1e\x0b\xb0\x0a", 0)
  time.sleep(0.1)
  print("flashing", file)
  flush_panda()
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


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='Flash body over can')
  parser.add_argument("board", type=str, nargs='?', help="choose base or knee")
  parser.add_argument("fn", type=str, nargs='?', help="flash file")
  args = parser.parse_args()

  assert args.board in ["base", "knee"]
  assert os.path.isfile(args.fn)

  addr = 0x250 if args.board == "base" else 0x350

  p = Panda()
  _thread.start_new_thread(heartbeat_thread, (p,))
  p.set_safety_mode(CarParams.SafetyModel.body)

  print("Flashing motherboard")
  flasher(p, addr, args.fn)

  print("CAN flashing done")
