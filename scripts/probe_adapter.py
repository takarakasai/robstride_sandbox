#!/usr/bin/env python3
"""
USB-CAN adapter protocol probe script.
Tries common Chinese USB-CAN adapter protocols to identify what this adapter speaks.
"""
import serial
import time
import struct
import sys

DEVICE = "/dev/ttyUSB0"

def try_baud(port_path, baud, probe_data, label, wait=0.5):
    """Send probe data at given baud rate and show any response."""
    try:
        ser = serial.Serial(port_path, baud, timeout=wait)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.1)
        
        # Flush any pending data
        if ser.in_waiting:
            old = ser.read(ser.in_waiting)
            print(f"  [pre-flush {len(old)} bytes: {old.hex()}]")
        
        ser.write(probe_data)
        ser.flush()
        time.sleep(wait)
        
        resp = b""
        if ser.in_waiting:
            resp = ser.read(ser.in_waiting)
        
        ser.close()
        
        if resp:
            print(f"  {label} @ {baud}baud: RESPONSE ({len(resp)} bytes): {resp.hex()}")
            print(f"    ASCII: {repr(resp)}")
            return True
        else:
            print(f"  {label} @ {baud}baud: no response")
            return False
    except Exception as e:
        print(f"  {label} @ {baud}baud: ERROR: {e}")
        return False

def probe_all():
    print(f"=== USB-CAN Adapter Protocol Probe ===")
    print(f"Device: {DEVICE}")
    print()
    
    # First, just listen for any spontaneous data at common baud rates
    print("--- Phase 1: Listen for spontaneous data ---")
    for baud in [115200, 921600, 2000000, 1000000, 500000, 460800, 256000]:
        try:
            ser = serial.Serial(DEVICE, baud, timeout=1.0)
            ser.reset_input_buffer()
            time.sleep(1.0)
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                print(f"  {baud}baud: GOT DATA ({len(data)} bytes): {data[:64].hex()}")
                ser.close()
                continue
            ser.close()
            print(f"  {baud}baud: silent")
        except Exception as e:
            print(f"  {baud}baud: error: {e}")
    
    print()
    print("--- Phase 2: SLCAN protocol ---")
    for baud in [115200, 921600, 2000000, 1000000, 500000]:
        # SLCAN Version
        try_baud(DEVICE, baud, b"V\r", "SLCAN-V")
        # SLCAN Close + Open + Set bitrate
        try_baud(DEVICE, baud, b"C\rS8\rO\r", "SLCAN-init")
    
    print()
    print("--- Phase 3: Chuangxin/CANalyst protocol (0xAA header) ---")
    for baud in [115200, 921600, 2000000, 1000000]:
        # Common init: AA 55 12 ...
        probe = bytes([0xAA, 0x55, 0x12, 0x01, 0x00, 0x00, 0x00, 0x00, 
                        0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 
                        0x00, 0x00, 0x00, 0x00])
        try_baud(DEVICE, baud, probe, "CANalyst-init")
    
    print()
    print("--- Phase 4: USB-CAN binary (header + CAN frame) ---")
    for baud in [115200, 921600, 2000000, 1000000, 500000]:
        # Protocol variant 1: 0xAA + FrameInfo + ID(4) + Data(8) + 0x55
        frame_info = 0x88  # extended, DLC=8
        can_id = struct.pack(">I", 0x0000FF01)  # GET_DEVICE_ID for motor 1
        data = bytes(8)
        checksum = sum([0xAA, frame_info] + list(can_id) + list(data)) & 0xFF
        probe = bytes([0xAA, frame_info]) + can_id + data + bytes([0x55])
        try_baud(DEVICE, baud, probe, "BIN-v1")
        
        # Protocol variant 2: Inverted header
        probe2 = bytes([0xAA, 0xC8]) + can_id + data + bytes([0x55])
        try_baud(DEVICE, baud, probe2, "BIN-v2")
    
    print()
    print("--- Phase 5: AT command protocol ---")
    for baud in [115200, 9600, 38400]:
        try_baud(DEVICE, baud, b"AT\r\n", "AT")
        try_baud(DEVICE, baud, b"AT+VERSION\r\n", "AT+VER")
    
    print()
    print("--- Phase 6: Raw binary CAN frame (no header) ---")
    for baud in [115200, 921600, 2000000, 1000000]:
        # Some adapters: just 4-byte EXT ID + 1-byte DLC + data
        can_id = struct.pack("<I", 0x0000FF01)  # little endian
        probe = can_id + bytes([8]) + bytes(8)
        try_baud(DEVICE, baud, probe, "RAW-LE")
        
        can_id_be = struct.pack(">I", 0x0000FF01)  # big endian
        probe = can_id_be + bytes([8]) + bytes(8)
        try_baud(DEVICE, baud, probe, "RAW-BE")

    print()
    print("=== Probe complete ===")

if __name__ == "__main__":
    probe_all()
