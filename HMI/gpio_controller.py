"""
GPIO / PWM output handling for the valve hardware.

Hardware assignment used in this implementation:
- Purge valve    -> GPIO27 -> DIO0 -> OUT0
- Maintain valve -> GPIO26 -> DIO1 -> OUT1
- Shared PWM     -> GPIO19  (PWM1, true hardware DMA via pigpio)
- Hardware E-stop input (active-low) -> GPIO22

Notes:
- If the GPIO library is unavailable, this module degrades gracefully.
- The shared PWM line is used to save power:
  1) 100% duty for a short kick pulse when the active valve changes
  2) lower hold duty after the kick
"""

from __future__ import annotations

import threading

try:
    import RPi.GPIO as GPIO
    import pigpio
    GPIO_AVAILABLE = True
except Exception:
    GPIO = None
    pigpio = None
    GPIO_AVAILABLE = False


class GPIOValveController:
    """Owns all valve-related GPIO outputs and the hardware E-stop input."""

    PWM_PIN = 19
    ESTOP_PIN = 22
    DIO0_PIN = 27
    DIO1_PIN = 26
    DIO2_PIN = 25
    DIO3_PIN = 24

    PWM_FREQUENCY_HZ = 20000
    KICK_DUTY = 1_000_000
    HOLD_DUTY = 350_000
    KICK_TIME_S = 0.250

    def __init__(self, log):
        self._log = log
        self._initialized = False
        self.pi = None
        self._pwm = None
        
        self.lock = threading.Lock()
        self._last_valves = {"dio0": False, "dio1": False, "dio2": False, "dio3": False}
        self._pwm_timer = None

    def initialize(self) -> None:
        """Set up GPIO pins and start PWM in a safe OFF state."""
        if not GPIO_AVAILABLE:
            self._log("GPIO library not available; valve actuation disabled.")
            return

        try:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)

            # Valve select outputs.
            GPIO.setup(self.DIO0_PIN, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.DIO1_PIN, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.DIO2_PIN, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.DIO3_PIN, GPIO.OUT, initial=GPIO.LOW)

            # Shared PWM output to the MOSFET board.
            self.pi = pigpio.pi()
            if not self.pi.connected:
                raise Exception("pigpiod daemon not running")
            
            self.pi.hardware_PWM(self.PWM_PIN, self.PWM_FREQUENCY_HZ, 0)

            # Physical E-stop is active-low according to the wiring notes.
            GPIO.setup(self.ESTOP_PIN, GPIO.IN, pull_up_down=GPIO.PUD_OFF)

            self._initialized = True
            self._log(
                "GPIO valve controller initialized "
                f"(dio0=GPIO{self.DIO0_PIN}, dio1=GPIO{self.DIO1_PIN}, "
                f"pwm=GPIO{self.PWM_PIN}, estop=GPIO{self.ESTOP_PIN})"
            )
        except Exception as exc:
            self._initialized = False
            self._log(f"GPIO init failed; valve actuation disabled ({exc})")

    @property
    def is_active(self) -> bool:
        return self._initialized

    def read_hardware_estop(self) -> bool:
        """Return True when the physical E-stop line is low."""
        if not self._initialized:
            return False
            
        try:
            # Active low implies if input is LOW, E-Stop is engaged.
            return GPIO.input(self.ESTOP_PIN) == GPIO.LOW
        except Exception as exc:
            self._log(f"Failed to read hardware E-stop ({exc})")
            return False

    def apply(self, dio0_on: bool, dio1_on: bool, dio2_on: bool, dio3_on: bool, estop_active: bool) -> None:
        """
        Drive valve-select outputs and shared PWM.

        Behavior:
        - If E-stop is active -> force all outputs off
        - If any valve transitions from False -> True -> apply 100% kick for KICK_TIME_S
        - After kick -> hold at lower duty cycle to save power
        """
        if not self._initialized:
            return

        with self.lock:
            if estop_active:
                self._force_all_off_locked()
                return

            new_state = {"dio0": dio0_on, "dio1": dio1_on, "dio2": dio2_on, "dio3": dio3_on}
            any_on = any(new_state.values())

            if not any_on:
                self._force_all_off_locked()
                return

            # Determine if any valve turned ON that wasn't ON before
            turned_on = any(new_state[k] and not self._last_valves[k] for k in new_state)

            self._last_valves = new_state

            # Update GPIO channels
            try:
                GPIO.output(self.DIO0_PIN, GPIO.HIGH if dio0_on else GPIO.LOW)
                GPIO.output(self.DIO1_PIN, GPIO.HIGH if dio1_on else GPIO.LOW)
                GPIO.output(self.DIO2_PIN, GPIO.HIGH if dio2_on else GPIO.LOW)
                GPIO.output(self.DIO3_PIN, GPIO.HIGH if dio3_on else GPIO.LOW)

                if turned_on:
                    # Apply 100% kick
                    self.pi.hardware_PWM(self.PWM_PIN, self.PWM_FREQUENCY_HZ, self.KICK_DUTY)

                    # Cancel any existing timer
                    if self._pwm_timer is not None:
                        self._pwm_timer.cancel()
                        
                    # Start async timer to downgrade the duty cycle
                    self._pwm_timer = threading.Timer(self.KICK_TIME_S, self._downgrade_pwm)
                    self._pwm_timer.daemon = True
                    self._pwm_timer.start()
                
            except Exception as exc:
                self._log(f"Failed to switch valves ({exc})")

    def _downgrade_pwm(self):
        """Timer callback to reduce PWM to hold duty."""
        with self.lock:
            # Check if all valves were shut off suddenly
            if not any(self._last_valves.values()):
                return
            try:
                self.pi.hardware_PWM(self.PWM_PIN, self.PWM_FREQUENCY_HZ, self.HOLD_DUTY)
            except Exception as exc:
                self._log(f"PWM hold update failed ({exc})")

    def _force_all_off_locked(self) -> None:
        """Internal worker to force off assuming lock is held."""
        try:
            if self._pwm_timer is not None:
                self._pwm_timer.cancel()
                self._pwm_timer = None
        except Exception:
            pass

        try:
            GPIO.output(self.DIO0_PIN, GPIO.LOW)
            GPIO.output(self.DIO1_PIN, GPIO.LOW)
            GPIO.output(self.DIO2_PIN, GPIO.LOW)
            GPIO.output(self.DIO3_PIN, GPIO.LOW)
            if self.pi is not None and self.pi.connected:
                self.pi.hardware_PWM(self.PWM_PIN, self.PWM_FREQUENCY_HZ, 0)
            self._last_valves = {"dio0": False, "dio1": False, "dio2": False, "dio3": False}
        except Exception as exc:
            self._log(f"Failed to force valve outputs off ({exc})")

    def _force_all_off(self) -> None:
        """Immediately shut off PWM and both valve-select outputs."""
        if not self._initialized:
            return
        with self.lock:
            self._force_all_off_locked()

    def shutdown(self) -> None:
        """Leave the hardware in a safe OFF state and release GPIO resources."""
        if not self._initialized:
            return
        self._force_all_off()
        try:
            if self.pi is not None and self.pi.connected:
                self.pi.hardware_PWM(self.PWM_PIN, self.PWM_FREQUENCY_HZ, 0)
                self.pi.stop()
        except Exception as exc:
            self._log(f"pigpio shutdown warning ({exc})")
        finally:
            try:
                GPIO.cleanup([self.DIO0_PIN, self.DIO1_PIN, self.DIO2_PIN, self.DIO3_PIN, self.ESTOP_PIN])
            except Exception:
                pass
            self._initialized = False
