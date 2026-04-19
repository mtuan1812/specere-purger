"""
GPIO / PWM output handling for the valve hardware.

Hardware assignment used in this implementation:
- Purge valve    -> GPIO27 -> DIO0 -> OUT0
- Maintain valve -> GPIO26 -> DIO1 -> OUT1
- Shared PWM     -> GPIO20
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
    GPIO_AVAILABLE = True
except Exception:
    GPIO = None
    GPIO_AVAILABLE = False


class GPIOValveController:
    """Owns all valve-related GPIO outputs and the hardware E-stop input."""

    PURGE_PIN = 27
    MAINTAIN_PIN = 26
    PWM_PIN = 20
    ESTOP_PIN = 22

    PWM_FREQUENCY_HZ = 25
    KICK_DUTY = 100
    HOLD_DUTY = 35
    KICK_TIME_S = 0.250

    def __init__(self, log):
        self._log = log
        self._initialized = False
        self._pwm = None
        
        self.lock = threading.Lock()
        self._last_active_valve = None
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
            GPIO.setup(self.PURGE_PIN, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.MAINTAIN_PIN, GPIO.OUT, initial=GPIO.LOW)

            # Shared PWM output to the MOSFET board.
            GPIO.setup(self.PWM_PIN, GPIO.OUT, initial=GPIO.LOW)
            self._pwm = GPIO.PWM(self.PWM_PIN, self.PWM_FREQUENCY_HZ)
            self._pwm.start(0)

            # Physical E-stop is active-low according to the wiring notes.
            GPIO.setup(self.ESTOP_PIN, GPIO.IN, pull_up_down=GPIO.PUD_OFF)

            self._initialized = True
            self._log(
                "GPIO valve controller initialized "
                f"(purge=GPIO{self.PURGE_PIN}, maintain=GPIO{self.MAINTAIN_PIN}, "
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

    def apply(self, purge_on: bool, maintain_on: bool, estop_active: bool) -> None:
        """
        Drive valve-select outputs and shared PWM.

        Behavior:
        - If E-stop is active -> force all outputs off
        - If both valves are off -> outputs off, PWM 0%
        - If an active valve changes -> apply 100% kick for KICK_TIME_S
        - After kick -> hold at lower duty cycle to save power
        """
        if not self._initialized:
            return

        with self.lock:
            if estop_active:
                self._force_all_off_locked()
                return

            active_valve = None
            if purge_on and not maintain_on:
                active_valve = "purge"
            elif maintain_on and not purge_on:
                active_valve = "maintain"
            elif purge_on and maintain_on:
                # Safety fallback: never intentionally energize both channels at once.
                active_valve = "purge"

            if active_valve is None:
                self._force_all_off_locked()
                return

            # If switching valves, initialize the kick
            if active_valve != self._last_active_valve:
                self._last_active_valve = active_valve
                
                # Update GPIO channels
                try:
                    GPIO.output(self.PURGE_PIN, GPIO.HIGH if active_valve == "purge" else GPIO.LOW)
                    GPIO.output(self.MAINTAIN_PIN, GPIO.HIGH if active_valve == "maintain" else GPIO.LOW)
                    
                    # Apply 100% kick
                    self._pwm.ChangeDutyCycle(self.KICK_DUTY)
                    self._log(f"Valve transition -> {active_valve} (kick {self.KICK_DUTY}% for {int(self.KICK_TIME_S * 1000)} ms)")

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
            if self._last_active_valve is None:
                return
            try:
                self._pwm.ChangeDutyCycle(self.HOLD_DUTY)
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
            GPIO.output(self.PURGE_PIN, GPIO.LOW)
            GPIO.output(self.MAINTAIN_PIN, GPIO.LOW)
            self._pwm.ChangeDutyCycle(0)
            self._last_active_valve = None
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
            if self._pwm is not None:
                self._pwm.stop()
        except Exception as exc:
            self._log(f"GPIO shutdown warning ({exc})")
        finally:
            try:
                GPIO.cleanup([self.PURGE_PIN, self.MAINTAIN_PIN, self.PWM_PIN, self.ESTOP_PIN])
            except Exception:
                pass
            self._initialized = False
