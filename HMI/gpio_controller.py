"""
GPIO / PWM output handling for the valve hardware.

Hardware assignment used in this implementation:
- Purge valve    -> GPIO27 -> DIO0 -> OUT0
- Maintain valve -> GPIO26 -> DIO1 -> OUT1
- Shared PWM     -> GPIO19  (PWM1)
- Hardware E-stop input (active-low) -> GPIO22

Notes:
- If the gpiozero library is unavailable, this module degrades gracefully.
- The shared PWM line is used to save power:
  1) 100% duty for a short kick pulse when the active valve changes
  2) lower hold duty after the kick
"""

from __future__ import annotations

import threading

try:
    from gpiozero import DigitalOutputDevice, InputDevice
    GPIO_AVAILABLE = True
except Exception:
    GPIO_AVAILABLE = False

try:
    from rpi_hardware_pwm import HardwarePWM
    HW_PWM_AVAILABLE = True
except Exception:
    HW_PWM_AVAILABLE = False


class GPIOValveController:
    """Owns all valve-related GPIO outputs and the hardware E-stop input."""

    PWM_PIN = 19
    ESTOP_PIN = 22
    DIO0_PIN = 27
    DIO1_PIN = 26
    DIO2_PIN = 25
    DIO3_PIN = 24

    PWM_FREQUENCY_HZ = 20000
    KICK_DUTY = 1.0
    HOLD_DUTY = 0.35
    KICK_TIME_S = 0.250

    def __init__(self, log):
        self._log = log
        self._initialized = False
        
        self.lock = threading.Lock()
        self._last_valves = {"dio0": False, "dio1": False, "dio2": False, "dio3": False}
        self._pwm_timer = None

        self.pwm = None
        self.estop = None
        self.dio0 = None
        self.dio1 = None
        self.dio2 = None
        self.dio3 = None

    def initialize(self) -> None:
        """Set up GPIO pins and start PWM in a safe OFF state."""
        if not GPIO_AVAILABLE:
            self._log("gpiozero library not available; valve actuation disabled.")
            return

        try:
            # Valve select outputs.
            self.dio0 = DigitalOutputDevice(self.DIO0_PIN, initial_value=False)
            self.dio1 = DigitalOutputDevice(self.DIO1_PIN, initial_value=False)
            self.dio2 = DigitalOutputDevice(self.DIO2_PIN, initial_value=False)
            self.dio3 = DigitalOutputDevice(self.DIO3_PIN, initial_value=False)

            # Shared PWM output to the MOSFET board using rpi_hardware_pwm.
            # pwm_channel=1 typically maps to GPIO 19 or GPIO 13
            if HW_PWM_AVAILABLE:
                self.pwm = HardwarePWM(pwm_channel=1, hz=self.PWM_FREQUENCY_HZ)
                self.pwm.start(0)  # Start at 0% duty cycle
            else:
                self._log("rpi_hardware_pwm not available; fallback to DigitalOutput 100% duty on GPIO 19.")
                self.pwm = DigitalOutputDevice(self.PWM_PIN, initial_value=False)

            # Physical E-stop is active-low according to the wiring notes.
            self.estop = InputDevice(self.ESTOP_PIN, pull_up=False)

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
            # Active low implies if input is LOW (0), E-Stop is engaged.
            return self.estop.value == 0
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
                self.dio0.value = dio0_on
                self.dio1.value = dio1_on
                self.dio2.value = dio2_on
                self.dio3.value = dio3_on

                if turned_on:
                    # Apply 100% kick
                    if self.pwm:
                        if HW_PWM_AVAILABLE:
                            self.pwm.change_duty_cycle(self.KICK_DUTY * 100.0)
                        else:
                            self.pwm.value = True

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
                if self.pwm:
                    if HW_PWM_AVAILABLE:
                        self.pwm.change_duty_cycle(self.HOLD_DUTY * 100.0)
                    else:
                        self.pwm.value = True
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
            if self.dio0: self.dio0.value = False
            if self.dio1: self.dio1.value = False
            if self.dio2: self.dio2.value = False
            if self.dio3: self.dio3.value = False
            if self.pwm:
                if HW_PWM_AVAILABLE:
                    self.pwm.change_duty_cycle(0.0)
                else:
                    self.pwm.value = False
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
            if self.pwm:
                if HW_PWM_AVAILABLE:
                    self.pwm.stop()
                else:
                    self.pwm.close()
            if self.dio0: self.dio0.close()
            if self.dio1: self.dio1.close()
            if self.dio2: self.dio2.close()
            if self.dio3: self.dio3.close()
            if self.estop: self.estop.close()
        except Exception as exc:
            self._log(f"gpiozero shutdown warning ({exc})")
        finally:
            self._initialized = False

