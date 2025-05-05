import RPi.GPIO as GPIO

# Use the Broadcom (BCM) pin numbering
GPIO.setmode(GPIO.BCM)

# List of all GPIO pins on a Raspberry Pi (BCM numbering)
ALL_GPIO_PINS = [
    2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
    14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
    24, 25, 26, 27
]

def deactivate_all_gpio():
    for pin in ALL_GPIO_PINS:
        try:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
        except Exception as e:
            print(f"Could not deactivate GPIO pin {pin}: {e}")

    print("All GPIO pins set to input with no pull resistors.")

# Run the function
deactivate_all_gpio()

# Clean up GPIO settings
GPIO.cleanup()
