"""
EEPROM Programmer for AT28C256 via Raspberry Pi GPIO and Shift Registers
=========================================================================

This script assembles a 6502 assembly source file, extracts interrupt vector
addresses, inserts the necessary 6502 vectors into the binary, and writes it
to an AT28C256 EEPROM connected to the Raspberry Pi Zero via shift registers.

Features:
- Automatically assembles .s file using ca65 and ld65 (part of cc65)
- Parses the .map file to detect reset, NMI, and IRQ vectors
- Pads the binary to 32KB and inserts correct vectors
- Uses shift registers for data and address lines to reduce GPIO usage
- Writes to the EEPROM using bit-banged GPIO logic

Usage:
    python3 eeprom_writer.py your_program.s

Requirements:
- cc65 must be installed and available in your PATH (ca65, ld65)
- Raspberry Pi GPIO wired to:
    - 3 shift registers (2 for address, 1 for data)
    - Write Enable (WE) pin of the EEPROM
- Assembly file must include labels:
    reset:, nmi:, irq: (optional, fallback to reset if missing)

Example Assembly Code:
    .segment "CODE"
    .org $0600

    reset:
        SEI
        CLD
        JMP main

    nmi:
        RTI

    irq:
        RTI

    main:
        LDA #$42
        STA $6000
        JMP main

"""

import RPi.GPIO as GPIO
import time
import subprocess
import os
import sys

# EEPROM and GPIO setup
EEPROM_SIZE = 32768  # 32KB
SR_SER = 2
SR_CLK = 3
SR_LATCH = 4
WE = 17

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SR_SER, GPIO.OUT)
    GPIO.setup(SR_CLK, GPIO.OUT)
    GPIO.setup(SR_LATCH, GPIO.OUT)
    GPIO.setup(WE, GPIO.OUT)
    GPIO.output(SR_SER, GPIO.LOW)
    GPIO.output(SR_CLK, GPIO.LOW)
    GPIO.output(SR_LATCH, GPIO.LOW)
    GPIO.output(WE, GPIO.HIGH)

def shift_out(byte):
    for i in range(7, -1, -1):
        GPIO.output(SR_SER, (byte >> i) & 1)
        GPIO.output(SR_CLK, GPIO.HIGH)
        GPIO.output(SR_CLK, GPIO.LOW)

def set_address_data(address, data):
    addr_low = address & 0xFF
    addr_high = (address >> 8) & 0x7F
    shift_out(data)
    shift_out(addr_low)
    shift_out(addr_high)
    GPIO.output(SR_LATCH, GPIO.HIGH)
    GPIO.output(SR_LATCH, GPIO.LOW)

def write_byte(address, data):
    set_address_data(address, data)
    GPIO.output(WE, GPIO.LOW)
    time.sleep(0.0001)
    GPIO.output(WE, GPIO.HIGH)
    time.sleep(0.01)

def write_data(start_address, data_bytes):
    for offset, byte in enumerate(data_bytes):
        write_byte(start_address + offset, byte)

def insert_vectors(contents, reset_vector=0x0600, nmi_vector=0x0600, irq_vector=0x0600):
    while len(contents) < EEPROM_SIZE:
        contents.append(0xFF)
    contents[0x7FFA] = nmi_vector & 0xFF
    contents[0x7FFB] = (nmi_vector >> 8) & 0xFF
    contents[0x7FFC] = reset_vector & 0xFF
    contents[0x7FFD] = (reset_vector >> 8) & 0xFF
    contents[0x7FFE] = irq_vector & 0xFF
    contents[0x7FFF] = (irq_vector >> 8) & 0xFF

def cleanup():
    GPIO.cleanup()

def generate_linker_config(path="none.cfg"):
    if not os.path.exists(path):
        with open(path, "w") as f:
            f.write("""\
MEMORY {
    ZP: start = $0000, size = $0100, type = rw, file = "";
    RAM: start = $0000, size = $8000, type = rw, file = %O, fill = yes, fillval = $FF;
}
SEGMENTS {
    ZEROPAGE: load = ZP, type = zp;
    CODE: load = RAM, type = ro;
    DATA: load = RAM, type = rw;
}
""")
        print(f"[+] Created linker config: {path}")

def assemble_to_bin(asm_file, bin_file="program.bin", cfg_file="none.cfg"):
    base = os.path.splitext(asm_file)[0]
    obj_file = base + ".o"
    map_file = base + ".map"
    print(f"[+] Assembling: {asm_file}")
    subprocess.run(["ca65", asm_file, "-g", "-o", obj_file], check=True)
    print(f"[+] Linking with map output")
    subprocess.run(["ld65", obj_file, "-C", cfg_file, "-o", bin_file, "--mapfile", map_file], check=True)
    return map_file

def parse_map_file(map_file):
    vectors = {"reset": 0x0600, "nmi": 0x0600, "irq": 0x0600}
    with open(map_file) as f:
        for line in f:
            line = line.strip()
            for key in vectors:
                if line.startswith(key + ":") or line.startswith(key + " ="):
                    parts = line.replace(":", "=").split("=")
                    if len(parts) == 2:
                        try:
                            addr = int(parts[1].strip().replace("$", ""), 16)
                            vectors[key] = addr
                        except ValueError:
                            pass
    print(f"[+] Detected vector addresses: {vectors}")
    return vectors

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

    with open(bin_file, "rb") as f:
        contents = bytearray(f.read())

    insert_vectors(contents,
                   reset_vector=vectors["reset"],
                   nmi_vector=vectors["nmi"],
                   irq_vector=vectors["irq"])

    print("[+] Writing to EEPROM...")
    setup()
    try:
        write_data(0x0000, contents)
    finally:
        cleanup()
    print("[✓] Done.")

if __name__ == "__main__":
    main()
