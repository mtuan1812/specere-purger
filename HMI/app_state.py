
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
        self._csv_folder = os.path.join(BASE_DIR, "telemetry")
        os.makedirs(self._csv_folder, exist_ok=True)
        self._current_date_str = None
        self._csv_buffer = []
        self.sensor_backend = create_sensor_backend(self.state.valves, self.log)
        self._lan_ip = self._get_lan_ip()

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
                    csv.writer(f).writerow(["timestamp_iso","epoch_s","o2_pct","flow_slm","pressure_mbar","ppo2_mbar","temp_c","rh_pct","purge_valve","steady_valve","mode","auto_running","fault","estop"])
                    
        m = self.state.metrics
        row = [dt.isoformat(sep=" ", timespec="seconds"), int(now), m.o2_pct, m.flow_slm, m.pressure_mbar, m.ppo2, m.temp_c, m.rh_pct, int(self.state.valves.purge), int(self.state.valves.steady), self.state.mode, int(self.state.auto_running), int(self.state.fault), int(self.state.estop)]
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
            if self.state.metrics.temp_c is not None: self.state.metrics.temp_c = round(self.state.metrics.temp_c, 2)
            if self.state.metrics.rh_pct is not None: self.state.metrics.rh_pct = round(self.state.metrics.rh_pct, 2)
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
        self._flush_csv_buffer()
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
        with self.lock:
            self.state.csv_url = f"http://{self._lan_ip}:8000/telemetry/"
            self.state.console_text = "\n".join(self.console_lines[-120:])
            self.state.sensor_backend = self.sensor_backend.backend_name
            self.state.connected = self.sensor_backend.is_connected
            return self.state.to_api_dict(range_sec)
