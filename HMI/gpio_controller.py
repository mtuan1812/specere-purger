
"""
GPIO / PWM output handling for the valve hardware.

Hardware assignment used in this implementation:
- Purge valve    -> GPIO27 -> DIO0 -> OUT0
- Maintain valve -> GPIO26 -> DIO1 -> OUT1
- Shared PWM     -> GPIO20
- Hardware E-stop input (active-low) -> GPIO22

Notes:
- If the GPIO library is unavailable, this module degrades gracefully.
- The UI / backend still runs on development machines without Raspberry Pi GPIO.
- The shared PWM line is used to save power:
  1) 100% duty for a short kick pulse when the active valve changes
  2) lower hold duty after the kick
"""

from __future__ import annotations

import time

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
        self._last_active_valve = None
        self._kick_until = 0.0

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
            # External hardware already biases the line, so leave the internal pull disabled.
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
        """Return True when the physical E-stop line is low (engaged)."""
        if not self._initialized:
            return False
        try:
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

        if estop_active:
            self._force_all_off()
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
            self._force_all_off()
            return

        # Select the currently active valve channel.
        GPIO.output(self.PURGE_PIN, GPIO.HIGH if active_valve == "purge" else GPIO.LOW)
        GPIO.output(self.MAINTAIN_PIN, GPIO.HIGH if active_valve == "maintain" else GPIO.LOW)

        # Kick each time the active valve changes.
        now = time.monotonic()
        if active_valve != self._last_active_valve:
            self._last_active_valve = active_valve
            self._kick_until = now + self.KICK_TIME_S
            self._log(
                f"Valve transition -> {active_valve} "
                f"(kick {self.KICK_DUTY}% for {int(self.KICK_TIME_S * 1000)} ms, "
                f"then hold {self.HOLD_DUTY}%)"
            )

        duty = self.KICK_DUTY if now < self._kick_until else self.HOLD_DUTY
        try:
            self._pwm.ChangeDutyCycle(duty)
        except Exception as exc:
            self._log(f"PWM duty update failed ({exc})")

    def _force_all_off(self) -> None:
        """Immediately shut off PWM and both valve-select outputs."""
        if not self._initialized:
            return
        try:
            GPIO.output(self.PURGE_PIN, GPIO.LOW)
            GPIO.output(self.MAINTAIN_PIN, GPIO.LOW)
            self._pwm.ChangeDutyCycle(0)
            self._last_active_valve = None
            self._kick_until = 0.0
        except Exception as exc:
            self._log(f"Failed to force valve outputs off ({exc})")

    def shutdown(self) -> None:
        """Leave the hardware in a safe OFF state and release GPIO resources."""
        if not self._initialized:
            return
        try:
            self._force_all_off()
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
