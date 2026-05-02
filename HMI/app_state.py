
import csv, os, threading, time
from datetime import datetime
from models import HistoryPoint, UiState, ValveState
from sensor_backend import create_sensor_backend
from gpio_controller import GPIOValveController

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_PATH = os.path.join(BASE_DIR, "telemetry.csv")

HYSTERESIS_PCT = 0.25
O2_WATCHDOG_TIMEOUT_S = 10  # seconds without valid O2 before auto-mode safe-stops

class RuntimeState:
    """Main backend state container used by the web server and poller thread."""
    def __init__(self):
        self.lock = threading.Lock()
        self._start_time = time.time()
        self._console_lock = threading.Lock()
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
        try:
            self.sensor_backend = create_sensor_backend(self.log)
        except Exception as exc:
            self.sensor_backend = None
            self.log(f"Sensor backend init failed: {exc}")
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
        with self._console_lock:
            new_lines = self.console_lines + [f"[{stamp}] {text}"]
            self.console_lines = new_lines[-200:]

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

    def _append_csv_row(self, now, snap):
        dt = datetime.fromtimestamp(now)
        date_str = dt.strftime("%Y-%m-%d")
        path = os.path.join(self._csv_folder, f"{date_str}.csv")
        
        # New day detection for rotation
        if date_str != self._current_date_str:
            self._current_date_str = date_str
            threading.Thread(target=self._clean_old_csvs, args=(now,), daemon=True).start()
            if not os.path.exists(path):
                with open(path, "w", newline="", encoding="utf-8") as f:
                    csv.writer(f).writerow(["timestamp_iso","epoch_s","o2_pct","flow_slm","pressure_mbar","ppo2_mbar","temp_c","rh_pct","dio0","dio1","dio2","dio3","mode","faults"])

        # snap = (o2, flow, pressure, ppo2, temp, rh, dio0, dio1, dio2, dio3, mode, fault_msg)
        row = [dt.isoformat(sep=" ", timespec="seconds"), int(now),
               snap[0], snap[1], snap[2], snap[3], snap[4], snap[5],
               int(snap[6]), int(snap[7]), int(snap[8]), int(snap[9]),
               snap[10], snap[11]]
        self._csv_buffer.append((path, row))
        
        if len(self._csv_buffer) >= 120:
            self._flush_csv_buffer()

    def _flush_csv_buffer(self):
        if not self._csv_buffer: return
        buffer_copy = list(self._csv_buffer)
        self._csv_buffer.clear()
        threading.Thread(target=self._do_flush, args=(buffer_copy,), daemon=True).start()

    def _do_flush(self, buffer_copy):
        by_path = {}
        for p, r in buffer_copy:
            by_path.setdefault(p, []).append(r)
            
        for p, rows in by_path.items():
            try:
                with open(p, "a", newline="", encoding="utf-8") as f:
                    csv.writer(f).writerows(rows)
            except Exception as e:
                self.log(f"Error flushing to CSV: {e}")

    @staticmethod
    def _format_timestamp_now():
        try: return datetime.now().strftime("%-m/%-d/%y %-I:%M:%S %p")
        except ValueError: return datetime.now().strftime("%m/%d/%y %I:%M:%S %p").lstrip("0").replace("/0", "/")

    @staticmethod
    def _format_last_seen(epoch):
        if epoch is None: return "--"
        try: return datetime.fromtimestamp(epoch).strftime("%-I:%M:%S %p")
        except ValueError: return datetime.fromtimestamp(epoch).strftime("%I:%M:%S %p").lstrip("0")

    @staticmethod
    def _format_uptime(seconds: float) -> str:
        s = int(seconds)
        days, s = divmod(s, 86400)
        hours, s = divmod(s, 3600)
        minutes, s = divmod(s, 60)
        if days > 0:
            return f"{days}d {hours}h {minutes}m"
        elif hours > 0:
            return f"{hours}h {minutes}m {s}s"
        elif minutes > 0:
            return f"{minutes}m {s}s"
        else:
            return f"{s}s"

    def _apply_fault_logic(self, fault_messages):
        all_msgs = list(fault_messages)
        if self.state.estop:
            all_msgs.insert(0, "E-stop active")
        if all_msgs:
            self.state.fault = True
            self.state.fault_message = "; ".join(all_msgs)
        else:
            self.state.fault = False
            self.state.fault_message = ""

    def _apply_auto_mode_logic(self):
        """Handle auto-mode state machine transitions."""
        if self.state.estop:
            self.state.valves.all_off()
            return
        if self.state.mode == "manual":
            return
        if self.state.mode == "standby":
            self.state.valves.all_off()
            return
        # mode == "auto"
        o2 = self.state.metrics.o2_pct
        if o2 is None:
            stale = (self._last_seen_epoch is None or
                     (time.time() - self._last_seen_epoch) > O2_WATCHDOG_TIMEOUT_S)
            if stale:
                self.state.valves.all_off()
                self.state.mode = "standby"
                self.log("Auto mode: O2 sensor timeout — entering standby, valves off")
            return
        if o2 >= self.state.target_o2 + HYSTERESIS_PCT:
            # O2 above setpoint + hysteresis — purge (dio0 on, dio1 off)
            self.state.valves.dio0 = True
            self.state.valves.dio1 = False
        elif o2 <= self.state.target_o2 - HYSTERESIS_PCT:
            # O2 below setpoint - hysteresis — steady (dio0 off, dio1 on)
            self.state.valves.dio0 = False
            self.state.valves.dio1 = True
        # else: within hysteresis band — hold current valve state (one always on)

    def _refresh_status_strings(self):
        self.state.timestamp_str = self._format_timestamp_now()
        self.state.last_seen_str = self._format_last_seen(self._last_seen_epoch)
        self.state.uptime_str = self._format_uptime(time.time() - self._start_time)
        if self.state.estop: self.state.system_status = "E-Stop Active ↗"
        elif self.state.fault: self.state.system_status = "System Fault ↗"
        else: self.state.system_status = "System Normal ↗"

    def step_lox(self):
        """Called from LuminOxThread. Blocks on readline() ~1 Hz.
        Updates O2/ppO2/pressure, drives control loop, appends history.
        """
        if self.sensor_backend is None:
            with self.lock:
                self._lox_fault_messages = ["LOX: sensor backend not initialized"]
                self._apply_fault_logic(self._lox_fault_messages + self._i2c_fault_messages)
                self._refresh_status_strings()
            time.sleep(1.0)
            return
        try:
            result = self.sensor_backend.read_lox(self.log)
        except Exception as exc:
            with self.lock:
                self._lox_fault_messages = [f"LOX: {exc}"]
                self._apply_fault_logic(self._lox_fault_messages + self._i2c_fault_messages)
                self._refresh_status_strings()
            return
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
                temp=self.state.metrics.temp_c,
                rh=self.state.metrics.rh_pct,
            ))
            self.state.history = self.state.history[-90000:]
            self._refresh_status_strings()
            self.state.console_text = "\n".join(self.console_lines[-120:])
            # Snapshot state for CSV under the lock to avoid data race
            csv_snap = (
                self.state.metrics.o2_pct,
                self.state.metrics.flow_slm,
                self.state.metrics.pressure_mbar,
                self.state.metrics.ppo2,
                self.state.metrics.temp_c,
                self.state.metrics.rh_pct,
                self.state.valves.dio0,
                self.state.valves.dio1,
                self.state.valves.dio2,
                self.state.valves.dio3,
                self.state.mode,
                self.state.fault_message,
            )
        # CSV append/flush OUTSIDE the lock — file I/O must never hold self.lock.
        self._append_csv_row(now, csv_snap)

    def step_i2c(self, read_sht: bool = False):
        """Called from I2CThread at ~5 Hz. Patches only flow/temp/rh on metrics."""
        if self.sensor_backend is None:
            with self.lock:
                self._i2c_fault_messages = ["I2C: sensor backend not initialized"]
                self._apply_fault_logic(self._lox_fault_messages + self._i2c_fault_messages)
                self._refresh_status_strings()
            return
        try:
            result = self.sensor_backend.read_i2c(self.log, read_sht=read_sht)
        except Exception as exc:
            with self.lock:
                self._i2c_fault_messages = [f"I2C: {exc}"]
                self._apply_fault_logic(self._lox_fault_messages + self._i2c_fault_messages)
                self._refresh_status_strings()
            return
        with self.lock:
            self.state.metrics.flow_slm = result.flow_slm
            if read_sht:
                self.state.metrics.temp_c = round(result.temp_c, 2) if result.temp_c is not None else None
                self.state.metrics.rh_pct = round(result.rh_pct, 2) if result.rh_pct is not None else None
            self._i2c_fault_messages = result.fault_messages
            self._apply_fault_logic(self._lox_fault_messages + self._i2c_fault_messages)
            self._refresh_status_strings()
            self.state.console_text = "\n".join(self.console_lines[-120:])

    def step_gpio(self):
        estop = self.gpio.read_hardware_estop()
        with self.lock:
            if estop != self.state.estop:
                self.state.estop = estop
                if estop:
                    # Hardware E-stop engaged — force standby and close all valves
                    self.state.mode = "standby"
                    self.state.valves.all_off()
                    self.log("E-stop engaged: mode set to standby, all valves closed")
                self._apply_fault_logic(self._lox_fault_messages + self._i2c_fault_messages)
                self._refresh_status_strings()

            # Apply states. If estop is True, gpio_controller forces all to 0 hardware-wise
            self.gpio.apply(
                dio0_on=self.state.valves.dio0,
                dio1_on=self.state.valves.dio1,
                dio2_on=self.state.valves.dio2,
                dio3_on=self.state.valves.dio3,
                estop_active=estop
            )

    def shutdown(self):
        self.log("Shutting down runtime")
        self._flush_csv_buffer()
        if self.sensor_backend is not None:
            self.sensor_backend.shutdown(self.log)
        self.gpio.shutdown()

    def handle_command(self, action, data):
        with self.lock:
            if action == "set_mode":
                new_mode = data.get("mode")
                if new_mode == "manual":
                    self.state.mode = "manual"
                elif new_mode == "auto" and self.state.mode == "manual":
                    self.state.mode = "standby"
            elif action == "adjust_setpoint":
                delta = float(data.get("delta", 0))
                self.state.target_o2 = min(25.0, max(0.1, round(self.state.target_o2 + delta, 1)))
            elif action == "toggle_auto_running":
                if self.state.mode == "auto":
                    self.state.mode = "standby"
                    self.state.valves.all_off()
                elif self.state.mode == "standby":
                    self.state.mode = "auto"
                    # Pick initial valve immediately so one is always on from the start.
                    o2 = self.state.metrics.o2_pct
                    if o2 is None or o2 >= self.state.target_o2:
                        # O2 at/above target (or unknown) — start purging
                        self.state.valves.dio0 = True
                        self.state.valves.dio1 = False
                    else:
                        # O2 below target — start steady
                        self.state.valves.dio0 = False
                        self.state.valves.dio1 = True
            elif action == "toggle_valve":
                if not self.state.locked_controls and not self.state.estop:
                    valve = data.get("valve")
                    if valve == "dio0": self.state.valves.dio0 = not self.state.valves.dio0
                    elif valve == "dio1": self.state.valves.dio1 = not self.state.valves.dio1
                    elif valve == "dio2": self.state.valves.dio2 = not self.state.valves.dio2
                    elif valve == "dio3": self.state.valves.dio3 = not self.state.valves.dio3
            elif action == "close_app":
                # Force standby and close all valves before shutdown
                self.state.mode = "standby"
                self.state.valves.all_off()
                self.state.estop = True
                self._apply_fault_logic(self._lox_fault_messages + self._i2c_fault_messages)
                self.log("E-stop pressed: mode set to standby, all valves closed")
                self.log("Executing system shutdown script...")
                import subprocess
                subprocess.Popen(["sudo", "sh", os.path.join(BASE_DIR, "stop_hmi.sh")], start_new_session=True)
            elif action == "shutdown_system":
                self.state.mode = "standby"
                self.state.valves.all_off()
                self.state.estop = True
                self._apply_fault_logic(self._lox_fault_messages + self._i2c_fault_messages)
                self.log("Shutting down Raspberry Pi...")
                import subprocess
                subprocess.Popen(["sudo", "shutdown", "now"])
            elif action == "reboot_system":
                self.log("Rebooting Raspberry Pi...")
                import subprocess
                subprocess.Popen(["sudo", "reboot"])
            elif action == "toggle_dim":
                self.log("Putting display to sleep (wake on touch)...")
                # Add a delay so the touch release event doesn't immediately wake the screen
                def dim_display():
                    import time
                    import subprocess
                    time.sleep(0.5)
                    subprocess.Popen([os.path.join(BASE_DIR, "screen_manager.sh"), "dim"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                threading.Thread(target=dim_display, daemon=True).start()
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
                "target_o2":      s.target_o2,
                "valves":         s.valves.to_dict(),
                "gpio_ok":        self.gpio.is_active,
                "estop":          s.estop,
                "fault":          s.fault,
                "fault_message":  s.fault_message,
                "connected":      self.sensor_backend.is_connected if self.sensor_backend else False,
                "locked_controls":s.locked_controls,
                "dimmed":         s.dimmed,
                "timestamp_str":  s.timestamp_str,
                "last_seen_str":  s.last_seen_str,
                "staleness_sec":  time.time() - self._last_seen_epoch if self._last_seen_epoch else None,
                "uptime_str":     s.uptime_str,
                "system_status":  s.system_status,
                "metrics":        s.metrics.to_dict(),
                "console_text":   "\n".join(self.console_lines[-120:]),
                "csv_url":        f"http://{self._lan_ip}:8000/telemetry/",
                "sensor_backend": self.sensor_backend.backend_name if self.sensor_backend else "none",
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
