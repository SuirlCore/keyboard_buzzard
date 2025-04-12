#program to listen for gpio ports connected to key grid

import RPi.GPIO as GPIO
import time

# === CONFIGURATION ===
DATA_OUT_PIN = 17   # GPIO pin to shift data OUT to column shift registers
CLOCK_PIN = 27      # Shared clock pin for both IN and OUT shift registers
LATCH_OUT_PIN = 22  # Latch pin for columns (output)
LATCH_IN_PIN = 13   # Latch pin for rows (input)
DATA_IN_PIN = 5     # Data IN pin from row shift register

NUM_COLUMNS = 22
NUM_ROWS = 5

DEBOUNCE_TIME = 0.02  # 20ms debounce

# === KEYMAP ===
# Map (column, row) -> key name
keymap = {
    (0, 0): "ESC",
    (1, 0): "1",
    (2, 0): "2",
    (3, 0): "3",
    (4, 0): "4",
    (5, 0): "5",
    (6, 0): "6",
    (7, 0): "7",
    (8, 0): "8",
    (9, 0): "9",
    (10, 0): "0",
    (11, 0): "-",
    (12, 0): "=",
    (13, 0): "BACKSPACE",
    (14, 0): "TAB",
    (15, 0): "Q",
    (16, 0): "W",
    (17, 0): "E",
    (18, 0): "R",
    (19, 0): "T",
    (20, 0): "Y",
    (21, 0): "U",
    # Add rows 1–4 mappings similarly...
    # Example:
    # (0, 1): "A",
    # (1, 1): "S",
    # etc.
}

# === SETUP ===
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Outputs
GPIO.setup(DATA_OUT_PIN, GPIO.OUT)
GPIO.setup(CLOCK_PIN, GPIO.OUT)
GPIO.setup(LATCH_OUT_PIN, GPIO.OUT)
GPIO.setup(LATCH_IN_PIN, GPIO.OUT)

# Input
GPIO.setup(DATA_IN_PIN, GPIO.IN)

# === FUNCTIONS ===

def shift_out(bits):
    """Shift out bits to the output shift register (columns)"""
    GPIO.output(LATCH_OUT_PIN, GPIO.LOW)
    for bit in bits:
        GPIO.output(DATA_OUT_PIN, bit)
        GPIO.output(CLOCK_PIN, GPIO.HIGH)
        GPIO.output(CLOCK_PIN, GPIO.LOW)
    GPIO.output(LATCH_OUT_PIN, GPIO.HIGH)

def shift_in(num_bits):
    """Shift in bits from the input shift register (rows)"""
    GPIO.output(LATCH_IN_PIN, GPIO.LOW)
    GPIO.output(LATCH_IN_PIN, GPIO.HIGH)
    bits = []
    for _ in range(num_bits):
        GPIO.output(CLOCK_PIN, GPIO.HIGH)
        bit = GPIO.input(DATA_IN_PIN)
        bits.append(bit)
        GPIO.output(CLOCK_PIN, GPIO.LOW)
    return bits

def clear_columns():
    """Set all columns inactive (high)"""
    shift_out([1] * NUM_COLUMNS)

def scan_matrix():
    """Scan the whole keyboard matrix and return a list of pressed keys"""
    pressed_keys = []

    for col in range(NUM_COLUMNS):
        bits = [1] * NUM_COLUMNS
        bits[col] = 0  # Activate one column (active-low)

        shift_out(bits)
        time.sleep(0.001)  # Small settle time

        rows = shift_in(NUM_ROWS)
        for row in range(NUM_ROWS):
            if rows[row] == 0:  # Active low: key pressed
                pressed_keys.append((col, row))

    clear_columns()
    return pressed_keys

# === MAIN LOOP ===

try:
    clear_columns()
    print("Starting keyboard scan...")
    last_keys = set()

    while True:
        keys = set(scan_matrix())

        if keys != last_keys:
            new_keys = keys - last_keys
            released_keys = last_keys - keys

            for col, row in new_keys:
                keyname = keymap.get((col, row), f"({col},{row})")
                print(f"Key pressed: {keyname}")

            for col, row in released_keys:
                keyname = keymap.get((col, row), f"({col},{row})")
                print(f"Key released: {keyname}")

            last_keys = keys

        time.sleep(DEBOUNCE_TIME)

except KeyboardInterrupt:
    print("Exiting...")

finally:
    clear_columns()
    GPIO.cleanup()
