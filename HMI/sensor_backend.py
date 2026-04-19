
import importlib.util, math, os, time
from dataclasses import dataclass
from typing import List, Optional
from models import TelemetryData

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
TEST_PATH = os.path.join(BASE_DIR, "test.py")

def load_test_module():
    try:
        spec = importlib.util.spec_from_file_location("sensor_test_module", TEST_PATH)
        if spec is None or spec.loader is None:
            raise RuntimeError("Could not create module spec for test.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module, None
    except Exception as exc:
        return None, exc

@dataclass
class BackendLoxResult:
    """Result of a single LuminOx UART read (~1 Hz, blocking)."""
    o2_pct: Optional[float]
    ppo2_mbar: Optional[float]
    pressure_mbar: Optional[float]
    fault_messages: List[str]

@dataclass
class BackendI2cResult:
    """Result of a single I2C read cycle (SFM4300 always; SHT45 optional)."""
    flow_slm: Optional[float]
    temp_c: Optional[float]   # None when SHT45 was skipped this cycle
    rh_pct: Optional[float]   # None when SHT45 was skipped this cycle
    fault_messages: List[str]

class SensorBackend:
    def initialize(self, log): raise NotImplementedError
    def read_lox(self, log) -> BackendLoxResult: raise NotImplementedError
    def read_i2c(self, log, read_sht: bool = False) -> BackendI2cResult: raise NotImplementedError
    def shutdown(self, log): raise NotImplementedError
    @property
    def is_connected(self): raise NotImplementedError
    @property
    def backend_name(self): raise NotImplementedError

class SimulatedSensorBackend(SensorBackend):
    def __init__(self, valve_state):
        self._valve_state = valve_state

    def initialize(self, log): log("Simulation backend active")

    def read_lox(self, log) -> BackendLoxResult:
        """Simulates the 1 Hz UART stream by sleeping 1 s."""
        time.sleep(1.0)
        t = time.time()
        o2 = max(0.1, 2.2 + 0.6 * math.sin(t / 18.0)
                      - (0.8 if self._valve_state.purge else 0)
                      - (0.3 if self._valve_state.steady else 0))
        return BackendLoxResult(
            o2_pct=round(o2, 2),
            ppo2_mbar=round(max(0, o2 / 100.0 * 1013), 1),
            pressure_mbar=round(1013 + 4 * math.sin(t / 30.0), 2),
            fault_messages=[],
        )

    def read_i2c(self, log, read_sht: bool = False) -> BackendI2cResult:
        """Returns immediately — the I2C thread's deadline scheduler controls pacing."""
        t = time.time()
        flow = (5.5 if self._valve_state.purge else 0.0) + (0.7 if self._valve_state.steady else 0.0)
        return BackendI2cResult(
            flow_slm=round(flow, 2),
            temp_c=round(24 + 1.0 * math.sin(t / 40.0), 2) if read_sht else None,
            rh_pct=round(48 + 8 * math.sin(t / 55.0), 2) if read_sht else None,
            fault_messages=[],
        )

    def shutdown(self, log): log("Simulation backend shutdown")
    @property
    def is_connected(self): return False
    @property
    def backend_name(self): return "sim"

class TestPySensorBackend(SensorBackend):
    def __init__(self, test_mod):
        self.test_mod = test_mod
        self.bus = None
        self.ser = None
        self.connected = False

    def initialize(self, log):
        self.bus = self.test_mod.smbus2.SMBus(self.test_mod.I2C_BUS)
        self.ser = self.test_mod.serial.Serial(
            port=self.test_mod.UART_PORT,
            baudrate=self.test_mod.UART_BAUD,
            bytesize=self.test_mod.serial.EIGHTBITS,
            parity=self.test_mod.serial.PARITY_NONE,
            stopbits=self.test_mod.serial.STOPBITS_ONE,
            timeout=2.1,
        )
        self.ser.write(b"M 0\r\n")
        time.sleep(0.5)
        self.ser.reset_input_buffer()
        # Reset the SFM4300 start flag so the new bus handle re-sends the start command.
        self.test_mod.sfm4300_reset()
        self.connected = True
        log("Sensor backend initialized from test.py")

    def read_lox(self, log) -> BackendLoxResult:
        """Blocks until a fresh LuminOx line arrives (~1 Hz via readline())."""
        try:
            lox = self.test_mod.luminox_read_line(self.ser)
            faults = []
            if "error" in lox:
                faults.append(f"LOX: {lox['error']}")
            elif "status_ok" in lox and not lox.get("status_ok", False):
                faults.append(f"LuminOx status {lox.get('status')}")
            log(f"O2 status: e {lox.get('status', 'ERR')}")
            return BackendLoxResult(
                o2_pct=None if "error" in lox else lox.get("o2_pct"),
                ppo2_mbar=None if "error" in lox else lox.get("ppo2_mbar"),
                pressure_mbar=None if "error" in lox else lox.get("pressure_mbar"),
                fault_messages=faults,
            )
        except Exception as exc:
            self.connected = False
            log(f"LuminOx read failure ({exc})")
            raise

    def read_i2c(self, log, read_sht: bool = False) -> BackendI2cResult:
        """Reads SFM4300 (always) and optionally SHT45. Typically <10 ms total."""
        try:
            sfm = self.test_mod.sfm4300_read(self.bus)
            log(f"Flow status: {sfm.get('status', 'N/A')}")

            sht: dict = {}
            if read_sht:
                sht = self.test_mod.sht45_read(self.bus)
                sht_status = "OK" if "error" not in sht else f"ERR ({sht['error']})"
                log(f"SHT45: {sht_status}")

            faults = []
            if "error" in sfm: faults.append(f"SFM4300: {sfm['error']}")
            if "error" in sht: faults.append(f"SHT45: {sht['error']}")

            return BackendI2cResult(
                flow_slm=None if "error" in sfm else sfm.get("flow_slm"),
                temp_c=(None if (not read_sht or "error" in sht) else sht.get("temp_c")),
                rh_pct=(None if (not read_sht or "error" in sht) else sht.get("rh_pct")),
                fault_messages=faults,
            )
        except Exception as exc:
            self.connected = False
            log(f"I2C read failure ({exc})")
            raise

    def shutdown(self, log):
        try:
            if self.bus is not None:
                try:
                    self.bus.write_i2c_block_data(self.test_mod.SFM4300_ADDR, self.test_mod.SFM4300_CMD_STOP[0], self.test_mod.SFM4300_CMD_STOP[1:])
                    time.sleep(0.05)
                except Exception as exc:
                    log(f"SFM stop failed during cleanup: {exc}")
                try:
                    self.bus.close()
                except Exception as exc:
                    log(f"I2C bus close failed: {exc}")
        finally:
            self.bus = None
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception as exc:
                log(f"Serial close failed: {exc}")
            finally:
                self.ser = None
        self.connected = False

    @property
    def is_connected(self): return self.connected
    @property
    def backend_name(self): return "test.py"

def create_sensor_backend(valve_state, log):
    test_mod, test_mod_error = load_test_module()
    if test_mod is None:
        log(f"test.py unavailable on this system; using simulation ({test_mod_error})")
        backend = SimulatedSensorBackend(valve_state)
        backend.initialize(log)
        return backend
    try:
        backend = TestPySensorBackend(test_mod)
        backend.initialize(log)
        return backend
    except Exception as exc:
        log(f"Sensor backend unavailable; using simulation ({exc})")
        backend = SimulatedSensorBackend(valve_state)
        backend.initialize(log)
        return backend
