#!/usr/bin/env python3
"""
EEPROM Writer for 6502 + AT28C256-15PU via Raspberry Pi GPIO
--------------------------------------------------------------

This script automates:
- Assembling a 6502 assembly (.s) program using ca65 and ld65.
- Verifying the memory map to ensure code fits in EEPROM ($C000–$FFFF).
- Filling vector locations (Reset, NMI, IRQ) automatically.
- Writing the final binary to the EEPROM via GPIO and shift registers.

How to Use:
-----------
1. Install cc65 (assembler and linker) on your Pi:
    sudo apt install cc65

2. Connect EEPROM:
    - Data lines: through shift register (serial out to EEPROM)
    - Address lines: through 2 shift registers (serial out to EEPROM)

3. Run:
    python3 eeprom_writer.py yourprogram.s

Requirements:
-------------
- Your .s file should define labels: `reset`, `nmi`, and `irq`.
- Assume code starts at $C000 (`.org $C000` or use linker config provided).

"""

import RPi.GPIO as GPIO
import time
import subprocess
import os
import sys

# GPIO Pin Configuration
DATA_SER = 17    # Data shift register serial input
DATA_SRCLK = 27  # Data shift register clock
DATA_RCLK = 22   # Data shift register latch

ADDR_SER = 23    # Address shift register serial input
ADDR_SRCLK = 24  # Address shift register clock
ADDR_RCLK = 25   # Address shift register latch

WE_PIN = 18      # EEPROM Write Enable

# EEPROM Settings
EEPROM_SIZE = 32 * 1024  # 32KB
EEPROM_BASE_ADDR = 0xC000  # EEPROM maps to $C000 - $FFFF

# Setup GPIO
def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup([DATA_SER, DATA_SRCLK, DATA_RCLK, ADDR_SER, ADDR_SRCLK, ADDR_RCLK, WE_PIN], GPIO.OUT)
    GPIO.output(WE_PIN, GPIO.HIGH)

def cleanup():
    GPIO.cleanup()

def pulse(pin):
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(0.000001)
    GPIO.output(pin, GPIO.LOW)
    time.sleep(0.000001)

def shift_out(data_pin, clock_pin, value, bits=8):
    for i in range(bits-1, -1, -1):
        GPIO.output(data_pin, (value >> i) & 0x01)
        pulse(clock_pin)

def write_byte(address, data):
    # Shift out address (16 bits)
    shift_out(ADDR_SER, ADDR_SRCLK, (address >> 8) & 0xFF)  # High byte
    shift_out(ADDR_SER, ADDR_SRCLK, address & 0xFF)         # Low byte
    pulse(ADDR_RCLK)

    # Shift out data (8 bits)
    shift_out(DATA_SER, DATA_SRCLK, data)
    pulse(DATA_RCLK)

    # Pulse WE to write
    GPIO.output(WE_PIN, GPIO.LOW)
    time.sleep(0.000001)
    GPIO.output(WE_PIN, GPIO.HIGH)
    time.sleep(0.0001)  # Allow EEPROM write cycle

def write_data(start_address, data):
    for i, byte in enumerate(data):
        write_byte(start_address + i, byte)

# Build Assembly
def assemble_to_bin(asm_file, bin_file, cfg_file):
    # Assemble
    result = subprocess.run(["ca65", asm_file, "-o", "program.o"])
    if result.returncode != 0:
        print("[!] Assembly failed.")
        sys.exit(1)

    # Link
    result = subprocess.run(["ld65", "program.o", "-C", cfg_file, "-o", bin_file, "--mapfile", "program.map"])
    if result.returncode != 0:
        print("[!] Linking failed.")
        sys.exit(1)

    return "program.map"

# Generate Linker Config
def generate_linker_config(cfg_file):
    with open(cfg_file, "w") as f:
        f.write("""MEMORY {
    ZP: start = $0000, size = $0100, type = rw;
    EEPROM: start = $C000, size = $4000, type = ro, file = %O, fill = yes, fillval = $FF;
}
SEGMENTS {
    ZEROPAGE: load = ZP, type = zp;
    CODE: load = EEPROM, type = ro;
    DATA: load = EEPROM, type = rw;
}
""")

# Parse Map File for Vectors
def parse_map_file(map_file):
    vectors = {"reset": None, "nmi": None, "irq": None}
    with open(map_file) as f:
        for line in f:
            for label in vectors.keys():
                if label in line:
                    parts = line.split()
                    if len(parts) > 2 and parts[0] == label:
                        vectors[label] = int(parts[2].replace("$", ""), 16)
    return vectors

# Verify Code Range
def verify_address_range(map_file):
    min_addr = 0xFFFF
    max_addr = 0x0000
    with open(map_file) as f:
        for line in f:
            line = line.strip()
            if "=" in line:
                parts = line.split("=")
                if len(parts) == 2:
                    try:
                        addr = int(parts[1].strip().replace("$", ""), 16)
                        if addr < min_addr:
                            min_addr = addr
                        if addr > max_addr:
                            max_addr = addr
                    except ValueError:
                        continue

    print(f"[+] Code address range: ${min_addr:04X} - ${max_addr:04X}")
    if min_addr < EEPROM_BASE_ADDR or max_addr > 0xFFFF:
        print("[!] WARNING: Code outside of EEPROM address range ($C000–$FFFF)!")
        print("[!] Aborting to prevent EEPROM corruption.")
        sys.exit(1)
    else:
        print("[✓] Address range is within EEPROM bounds.")

# Insert Vectors into Final Binary
def insert_vectors(contents, reset_vector, nmi_vector, irq_vector):
    while len(contents) < EEPROM_SIZE:
        contents.append(0xFF)

    # Set vector addresses
    def write_vector(addr, vector):
        contents[addr - EEPROM_BASE_ADDR] = vector & 0xFF
        contents[addr - EEPROM_BASE_ADDR + 1] = (vector >> 8) & 0xFF

    write_vector(0xFFFA, nmi_vector)
    write_vector(0xFFFC, reset_vector)
    write_vector(0xFFFE, irq_vector)

# Main
def main():
    if len(sys.argv) < 2:
        print("Usage: python3 eeprom_writer.py yourcode.s")
        return

    asm_file = sys.argv[1]
    bin_file = "program.bin"
    cfg_file = "none.cfg"

    generate_linker_config(cfg_file)
    map_file = assemble_to_bin(asm_file, bin_file, cfg_file)
    vectors = parse_map_file(map_file)
    verify_address_range(map_file)

    with open(bin_file, "rb") as f:
        contents = bytearray(f.read())

    insert_vectors(contents,
                   reset_vector=vectors["reset"],
                   nmi_vector=vectors["nmi"],
                   irq_vector=vectors["irq"])

    print("[+] Writing to EEPROM...")
    setup()
    try:
        write_data(0x0000, contents)  # EEPROM starts writing from address 0
    finally:
        cleanup()
    print("[✓] Done.")

if __name__ == "__main__":
    main()
