
import csv, os, threading, time
from datetime import datetime
from models import HistoryPoint, UiState, ValveState
from sensor_backend import create_sensor_backend
from gpio_controller import GPIOValveController

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_PATH = os.path.join(BASE_DIR, "telemetry.csv")

HYSTERESIS_PCT = 0.5

class RuntimeState:
    """Main backend state container used by the web server and poller thread."""
    def __init__(self):
        self.lock = threading.Lock()
        self.console_lines = []
        self.state = UiState()
        self.state.valves = ValveState()
        self._last_seen_epoch = None
        self._lox_fault_messages: list = []
        self._i2c_fault_messages: list = []
        self._csv_folder = os.path.join(BASE_DIR, "telemetry")
        os.makedirs(self._csv_folder, exist_ok=True)
        self._current_date_str = None
        self._csv_buffer = []
        self.sensor_backend = create_sensor_backend(self.state.valves, self.log)
        self._lan_ip = self._get_lan_ip()
        self.gpio = GPIOValveController(self.log)
        self.gpio.initialize()

    def _get_lan_ip(self):
        import socket
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(('8.8.8.8', 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "127.0.0.1"

    def log(self, text):
        stamp = datetime.now().strftime("%m/%d/%y %I:%M:%S %p")
        self.console_lines.append(f"[{stamp}] {text}")
        self.console_lines = self.console_lines[-200:]

    def _clean_old_csvs(self, now):
        try:
            for filename in os.listdir(self._csv_folder):
                if filename.endswith(".csv"):
                    file_path = os.path.join(self._csv_folder, filename)
                    # Parse date string to be accurate, or use file mtime. mtime is fine.
                    if os.path.isfile(file_path):
                        mtime = os.path.getmtime(file_path)
                        # 7 days * 86400 seconds = 604800
                        if now - mtime > 604800:
                            os.remove(file_path)
                            self.log(f"Cleaned up old log file: {filename}")
        except Exception as e:
            self.log(f"Error cleaning CSVs: {e}")

    def _append_csv_row(self, now):
        dt = datetime.fromtimestamp(now)
        date_str = dt.strftime("%Y-%m-%d")
        path = os.path.join(self._csv_folder, f"{date_str}.csv")
        
        # New day detection for rotation
        if date_str != self._current_date_str:
            self._current_date_str = date_str
            self._clean_old_csvs(now)
            if not os.path.exists(path):
                with open(path, "w", newline="", encoding="utf-8") as f:
                    csv.writer(f).writerow(["timestamp_iso","epoch_s","o2_pct","flow_slm","pressure_mbar","ppo2_mbar","temp_c","rh_pct","dio1_purge","dio2_steady","dio3","dio4","mode","auto_running","fault","estop"])
                    
        m = self.state.metrics
        row = [dt.isoformat(sep=" ", timespec="seconds"), int(now), m.o2_pct, m.flow_slm, m.pressure_mbar, m.ppo2, m.temp_c, m.rh_pct, int(self.state.valves.purge), int(self.state.valves.steady), int(self.state.valves.dio3), int(self.state.valves.dio4), self.state.mode, int(self.state.auto_running), int(self.state.fault), int(self.state.estop)]
        self._csv_buffer.append((path, row))
        
        if len(self._csv_buffer) >= 120:
            self._flush_csv_buffer()

    def _flush_csv_buffer(self):
        if not self._csv_buffer: return
        by_path = {}
        for p, r in self._csv_buffer:
            by_path.setdefault(p, []).append(r)
            
        for p, rows in by_path.items():
            try:
                with open(p, "a", newline="", encoding="utf-8") as f:
                    csv.writer(f).writerows(rows)
            except Exception as e:
                self.log(f"Error flushing to CSV: {e}")
                
        self._csv_buffer.clear()

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
        if o2 is not None:
            if o2 > self.state.target_o2 + HYSTERESIS_PCT:
                self.state.valves.purge = True
                self.state.valves.steady = False
            elif o2 < self.state.target_o2 - HYSTERESIS_PCT:
                self.state.valves.purge = False
                self.state.valves.steady = True
            # inside hysteresis band: leave valves as they are

    def _refresh_status_strings(self):
        self.state.timestamp_str = self._format_timestamp_now()
        self.state.last_seen_str = self._format_last_seen(self._last_seen_epoch)
        if self.state.estop: self.state.system_status = "E-Stop Active ↗"
        elif self.state.fault: self.state.system_status = "System Fault ↗"
        else: self.state.system_status = "System Normal ↗"

    def step_lox(self):
        """Called from LuminOxThread. Blocks on readline() ~1 Hz.
        Updates O2/ppO2/pressure, drives control loop, appends history.
        """
        try:
            result = self.sensor_backend.read_lox(self.log)
        except Exception:
            self.sensor_backend = create_sensor_backend(self.state.valves, self.log)
            result = self.sensor_backend.read_lox(self.log)
        now = time.time()
        with self.lock:
            self.state.metrics.o2_pct = result.o2_pct
            self.state.metrics.ppo2 = result.ppo2_mbar
            self.state.metrics.pressure_mbar = result.pressure_mbar
            self._lox_fault_messages = result.fault_messages
            if result.o2_pct is not None:
                self._last_seen_epoch = now
            self.state.connected = self.sensor_backend.is_connected
            self.state.sensor_backend = self.sensor_backend.backend_name
            self._apply_fault_logic(self._lox_fault_messages + self._i2c_fault_messages)
            self._apply_auto_mode_logic()
            # Snapshot current flow (written by I2C thread) into history point
            self.state.history.append(HistoryPoint(
                ts=now,
                o2=result.o2_pct,
                flow=self.state.metrics.flow_slm,
                pressure=result.pressure_mbar,
                ppo2=result.ppo2_mbar,
            ))
            self.state.history = self.state.history[-90000:]
            self._refresh_status_strings()
            self.state.console_text = "\n".join(self.console_lines[-120:])
        # CSV append/flush OUTSIDE the lock — file I/O must never hold self.lock.
        # _csv_buffer is only written by this thread (LuminOxThread) so no lock needed.
        self._append_csv_row(now)

    def step_i2c(self, read_sht: bool = False):
        """Called from I2CThread at ~5 Hz. Patches only flow/temp/rh on metrics.
        Does NOT own backend reinitialisation—that belongs to step_lox().
        """
        try:
            result = self.sensor_backend.read_i2c(self.log, read_sht=read_sht)
        except Exception as exc:
            with self.lock:
                self._i2c_fault_messages = [f"I2C: {exc}"]
            return
        with self.lock:
            if result.flow_slm is not None:
                self.state.metrics.flow_slm = result.flow_slm
            if read_sht:
                if result.temp_c is not None:
                    self.state.metrics.temp_c = round(result.temp_c, 2)
                if result.rh_pct is not None:
                    self.state.metrics.rh_pct = round(result.rh_pct, 2)
            self._i2c_fault_messages = result.fault_messages
            self._refresh_status_strings()
            self.state.console_text = "\n".join(self.console_lines[-120:])

    def step_gpio(self):
        estop = self.gpio.read_hardware_estop()
        with self.lock:
            if estop != self.state.estop:
                self.state.estop = estop
                self._refresh_status_strings()
            
            # Apply states. If estop is True, gpio_controller forces all to 0 hardware-wise
            self.gpio.apply(
                purge_on=self.state.valves.purge,
                steady_on=self.state.valves.steady,
                dio3_on=self.state.valves.dio3,
                dio4_on=self.state.valves.dio4,
                estop_active=estop
            )

    def shutdown(self):
        self.log("Shutting down runtime")
        self._flush_csv_buffer()
        self.sensor_backend.shutdown(self.log)
        self.gpio.shutdown()

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
            elif action == "toggle_valve":
                if not self.state.locked_controls and not self.state.estop:
                    valve = data.get("valve")
                    if valve == "purge": self.state.valves.purge = not self.state.valves.purge
                    elif valve == "steady": self.state.valves.steady = not self.state.valves.steady
                    elif valve == "dio3": self.state.valves.dio3 = not self.state.valves.dio3
                    elif valve == "dio4": self.state.valves.dio4 = not self.state.valves.dio4
            elif action == "toggle_estop":
                self.log("Executing system shutdown script...")
                import subprocess
                subprocess.Popen(["sudo", "sh", os.path.join(BASE_DIR, "stop_hmi.sh")], start_new_session=True)
            elif action == "reboot_system":
                self.log("Rebooting Raspberry Pi...")
                import subprocess
                subprocess.Popen(["sudo", "reboot"])
            elif action == "toggle_dim":
                self.log("Putting display to sleep (wake on touch)...")
                import subprocess
                # Allow xset to turn off the screen via DPMS. A touch event will automatically wake it.
                subprocess.Popen(["sudo", "-u", "admin", "env", "DISPLAY=:0", "XAUTHORITY=/home/admin/.Xauthority", "xset", "+dpms"])
                subprocess.Popen(["sudo", "-u", "admin", "env", "DISPLAY=:0", "XAUTHORITY=/home/admin/.Xauthority", "xset", "dpms", "force", "off"])
            elif action == "toggle_lock":
                self.state.locked_controls = not self.state.locked_controls
            self._refresh_status_strings()
            self.state.console_text = "\n".join(self.console_lines[-120:])

    def snapshot_dict(self, host=None, range_sec=1200):
        """Return the API dict for the frontend.

        Only scalar field reads and a shallow history copy happen under the lock.
        The expensive filter/serialisation of up to 90,000 HistoryPoints is done
        OUTSIDE the lock so sensor threads are not stalled during each 4 Hz poll.
        """
        with self.lock:
            s = self.state
            snap = {
                "mode":           s.mode,
                "auto_running":   s.auto_running,
                "target_o2":      s.target_o2,
                "valves":         s.valves.to_dict(),
                "estop":          s.estop,
                "fault":          s.fault,
                "fault_message":  s.fault_message,
                "connected":      self.sensor_backend.is_connected,
                "locked_controls":s.locked_controls,
                "dimmed":         s.dimmed,
                "timestamp_str":  s.timestamp_str,
                "last_seen_str":  s.last_seen_str,
                "system_status":  s.system_status,
                "metrics":        s.metrics.to_dict(),
                "console_text":   "\n".join(self.console_lines[-120:]),
                "csv_url":        f"http://{self._lan_ip}:8000/telemetry/",
                "sensor_backend": self.sensor_backend.backend_name,
            }
            history_snap = list(s.history)   # shallow copy — O(n) but no allocation per item
        # Expensive: filter + asdict on up to 90,000 points — done without the lock
        now = time.time()
        cutoff = now - range_sec
        recent = [p for p in history_snap if p.ts >= cutoff]
        if len(recent) > 1000:
            step = len(recent) // 500
            recent = recent[::step]
        snap["history"] = [p.to_dict() for p in recent]
        return snap
