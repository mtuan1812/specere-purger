
import csv, os, threading, time
from datetime import datetime
from models import HistoryPoint, UiState, ValveState
from sensor_backend import create_sensor_backend

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_PATH = os.path.join(BASE_DIR, "telemetry.csv")

class RuntimeState:
    """Main backend state container used by the web server and poller thread."""
    def __init__(self):
        self.lock = threading.Lock()
        self.console_lines = []
        self.state = UiState()
        self.state.valves = ValveState()
        self._last_seen_epoch = None
        self.sensor_backend = create_sensor_backend(self.state.valves, self.log)
        self._ensure_csv()

    def log(self, text):
        stamp = datetime.now().strftime("%m/%d/%y %I:%M:%S %p")
        self.console_lines.append(f"[{stamp}] {text}")
        self.console_lines = self.console_lines[-200:]

    def _ensure_csv(self):
        if not os.path.exists(CSV_PATH):
            with open(CSV_PATH, "w", newline="", encoding="utf-8") as f:
                csv.writer(f).writerow(["timestamp_iso","epoch_s","o2_pct","flow_slm","pressure_mbar","ppo2_mbar","temp_c","rh_pct","purge_valve","steady_valve","mode","auto_running","fault","estop"])

    def _append_csv_row(self, now):
        m = self.state.metrics
        with open(CSV_PATH, "a", newline="", encoding="utf-8") as f:
            csv.writer(f).writerow([datetime.fromtimestamp(now).isoformat(sep=" ", timespec="seconds"), int(now), m.o2_pct, m.flow_slm, m.pressure_mbar, m.ppo2, m.temp_c, m.rh_pct, int(self.state.valves.purge), int(self.state.valves.steady), self.state.mode, int(self.state.auto_running), int(self.state.fault), int(self.state.estop)])

    @staticmethod
    def _format_timestamp_now():
        try: return datetime.now().strftime("%-m/%-d/%y %-I:%M:%S %p")
        except ValueError: return datetime.now().strftime("%m/%d/%y %I:%M:%S %p").lstrip("0").replace("/0", "/")

    @staticmethod
    def _format_last_seen(epoch):
        if epoch is None: return "--"
        try: return datetime.fromtimestamp(epoch).strftime("%-I:%M:%S %p")
        except ValueError: return datetime.fromtimestamp(epoch).strftime("%I:%M:%S %p").lstrip("0")

    def _apply_fault_logic(self, fault_messages):
        if self.state.estop:
            self.state.fault = True
            self.state.fault_message = "E-stop active"
            return
        if fault_messages:
            self.state.fault = True
            self.state.fault_message = "; ".join(fault_messages)
        else:
            self.state.fault = False
            self.state.fault_message = ""

    def _apply_auto_mode_logic(self):
        """Placeholder auto-mode logic. Replace this block after weekend discussion."""
        if self.state.estop:
            self.state.valves.all_off()
            return
        if self.state.mode != "auto":
            return
        if not self.state.auto_running:
            self.state.valves.all_off()
            return
        o2 = self.state.metrics.o2_pct
        if o2 is not None and o2 > self.state.target_o2:
            self.state.valves.purge = (self.state.auto_path == "purge")
            self.state.valves.steady = (self.state.auto_path == "steady")
        else:
            self.state.valves.all_off()

    def _refresh_status_strings(self):
        self.state.timestamp_str = self._format_timestamp_now()
        self.state.last_seen_str = self._format_last_seen(self._last_seen_epoch)
        if self.state.estop: self.state.system_status = "E-Stop Active ↗"
        elif self.state.fault: self.state.system_status = "System Fault ↗"
        else: self.state.system_status = "System Normal ↗"

    def step(self):
        try:
            result = self.sensor_backend.read_once(self.log)
        except Exception:
            self.sensor_backend = create_sensor_backend(self.state.valves, self.log)
            result = self.sensor_backend.read_once(self.log)
        now = time.time()
        with self.lock:
            self.state.metrics = result.telemetry
            if result.telemetry.has_any_value():
                self._last_seen_epoch = now
            self.state.connected = self.sensor_backend.is_connected
            self.state.sensor_backend = self.sensor_backend.backend_name
            self._apply_fault_logic(result.fault_messages)
            self._apply_auto_mode_logic()
            self.state.history.append(HistoryPoint(ts=now, o2=result.telemetry.o2_pct, flow=result.telemetry.flow_slm, pressure=result.telemetry.pressure_mbar, ppo2=result.telemetry.ppo2))
            self.state.history = self.state.history[-90000:]
            self._append_csv_row(now)
            self._refresh_status_strings()
            self.state.console_text = "\n".join(self.console_lines[-120:])

    def shutdown(self):
        self.log("Shutting down runtime")
        self.sensor_backend.shutdown(self.log)

    def handle_command(self, action, data):
        with self.lock:
            if action == "set_mode":
                self.state.mode = "auto" if data.get("mode") == "auto" else "manual"
            elif action == "adjust_setpoint":
                delta = float(data.get("delta", 0))
                self.state.target_o2 = min(25.0, max(0.1, round(self.state.target_o2 + delta, 1)))
            elif action == "toggle_auto_running":
                self.state.auto_running = not self.state.auto_running
                if not self.state.auto_running: self.state.valves.all_off()
            elif action == "set_auto_path":
                self.state.auto_path = "steady" if data.get("path") == "steady" else "purge"
            elif action == "toggle_valve":
                if not self.state.locked_controls and not self.state.estop:
                    valve = data.get("valve")
                    if valve == "purge": self.state.valves.purge = not self.state.valves.purge
                    elif valve == "steady": self.state.valves.steady = not self.state.valves.steady
            elif action == "toggle_estop":
                self.state.estop = not self.state.estop
                if self.state.estop:
                    self.state.valves.all_off()
                    self.log("E-stop activated")
                else:
                    self.log("E-stop cleared")
            elif action == "reset_faults":
                if not self.state.estop:
                    self.state.fault = False
                    self.state.fault_message = ""
                self.log("Fault reset requested")
            elif action == "toggle_dim":
                self.state.dimmed = not self.state.dimmed
            elif action == "toggle_lock":
                self.state.locked_controls = not self.state.locked_controls
            self._refresh_status_strings()
            self.state.console_text = "\n".join(self.console_lines[-120:])

    def snapshot_dict(self, host):
        with self.lock:
            self.state.csv_url = f"http://{host}/telemetry.csv"
            self.state.console_text = "\n".join(self.console_lines[-120:])
            self.state.sensor_backend = self.sensor_backend.backend_name
            self.state.connected = self.sensor_backend.is_connected
            return self.state.to_api_dict()
