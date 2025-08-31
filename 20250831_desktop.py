import serial
import time
import os
from datetime import datetime
import struct  # add at top

PORT = "COM4"
BAUD = 12000000         # or 12000000 if STM32 is set that way
BUF_SIZE = 40000      # size of random test buffer

def read_exact(ser, n, timeout_s=5):
    """Read exactly n bytes or return what we got by timeout."""
    end = time.time() + timeout_s
    buf = bytearray()
    while len(buf) < n and time.time() < end:
        chunk = ser.read(n - len(buf))
        if chunk:
            buf += chunk
        else:
            time.sleep(0.001)
    return bytes(buf)

def timestamp_ms():
    """Return current time string with millisecond resolution."""
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]

try:
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    print(f"Connected to {PORT} at {BAUD} baud.\n")
    time.sleep(2)

    # ---- One-time loopback test ----
    msg = os.urandom(BUF_SIZE)
    t0 = time.time()
    ser.write(msg)
    ser.flush()
    echoed = read_exact(ser, BUF_SIZE, timeout_s=5)
    t1 = time.time()

    if len(echoed) == BUF_SIZE:
        rtt_ms = (t1 - t0) * 1000
        print(f"[Loopback] Received {len(echoed)} bytes, RTT {rtt_ms:.2f} ms")
        print(f"[Loopback] Integrity: {'Passed' if echoed == msg else 'FAILED'}")
    else:
        print(f"[Loopback] Incomplete ({len(echoed)}/{BUF_SIZE})")

    # Clear any leftover bytes before RX-only
    ser.reset_input_buffer()

    NUM_WORDS = 5000
    BYTES_PER_WORD = 4
    TOTAL_BYTES = NUM_WORDS * BYTES_PER_WORD

    while True:
        buf = bytearray()
        while len(buf) < TOTAL_BYTES:
            chunk = ser.read(TOTAL_BYTES - len(buf))  # read the remainder
            if not chunk:
                continue  # wait for more data
            buf.extend(chunk)

        # Decode as 10 little-endian uint32_t values
        values = struct.unpack(f"<{NUM_WORDS}I", buf)
        print(f"[{timestamp_ms()}] {values[0]}")


    # while True:
    #     data = ser.read(1)      # read a single byte
    #     if data:
    #         value = data[0]     # convert single byte to int
    #         print(f"[{timestamp_ms()}] {value}")
    #         ser.reset_input_buffer()  # flush any leftover bytes immediately


except serial.SerialException as e:
    print(f"Serial error: {e}")
except KeyboardInterrupt:
    print("\nStopped by user.")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed.")
