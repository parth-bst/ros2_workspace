#!/usr/bin/env python3
"""
Minimal LED Controller
Simple function to control LED on GPIO pin 21
"""

# GPIO imports for LED control
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except (ImportError, RuntimeError):
    try:
        import fake_rpi
        import sys
        sys.modules['RPi'] = fake_rpi.RPi
        sys.modules['RPi.GPIO'] = fake_rpi.RPi.GPIO
        import RPi.GPIO as GPIO
        GPIO_AVAILABLE = True
        print("Using fake_rpi for GPIO simulation")
    except ImportError:
        GPIO_AVAILABLE = False
        print("Warning: RPi.GPIO not available. LED control will be simulated.")

# Global state tracking
_gpio_initialized = False


def _init_gpio():
    """Initialize GPIO once"""
    global _gpio_initialized
    if not GPIO_AVAILABLE or _gpio_initialized:
        return
    
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        _gpio_initialized = True
        print("GPIO initialized for LED control")
    except Exception as e:
        print(f"Failed to initialize GPIO: {e}")


def light_led_on_gpio21():
    """
    Simple function to light up LED on GPIO pin 21
    """
    try:
        if not GPIO_AVAILABLE:
            print("Simulated: LED on GPIO 21 would be ON (GPIO not available)")
            return "Simulated LED on GPIO 21 turned ON"
        
        # Initialize GPIO if not already done
        _init_gpio()
        
        # Set up GPIO pin 21 as output and turn on LED
        GPIO.setup(21, GPIO.OUT)
        GPIO.output(21, GPIO.HIGH)
        
        print("LED on GPIO pin 21 turned ON")
        return "LED on GPIO pin 21 turned ON"
        
    except Exception as e:
        print(f"Error controlling LED: {e}")
        return f"Error controlling LED: {e}"


def init_led_controller():
    """Initialize LED controller for agent use"""
    _init_gpio()
    print("LED controller initialized")


def cleanup_gpio():
    """Cleanup GPIO resources"""
    global _gpio_initialized
    if GPIO_AVAILABLE and _gpio_initialized:
        try:
            GPIO.cleanup()
            _gpio_initialized = False
            print("GPIO cleanup completed")
        except Exception as e:
            print(f"GPIO cleanup error: {e}")


# Example usage
if __name__ == "__main__":
    init_led_controller()
    light_led_on_gpio21()
    cleanup_gpio()