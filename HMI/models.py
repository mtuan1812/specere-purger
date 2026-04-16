
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional
import time

@dataclass
class TelemetryData:
    """Latest numeric telemetry shown on the HMI."""
    o2_pct: Optional[float] = None
    flow_slm: Optional[float] = None
    pressure_mbar: Optional[float] = None
    ppo2: Optional[float] = None
    temp_c: Optional[float] = None
    rh_pct: Optional[float] = None
    def has_any_value(self) -> bool:
        return any(value is not None for value in asdict(self).values())
    def to_dict(self) -> Dict[str, Optional[float]]:
        return asdict(self)

@dataclass
class ValveState:
    """Logical state of the two controllable valves."""
    purge: bool = False
    steady: bool = False
    def all_off(self) -> None:
        self.purge = False
        self.steady = False
    def to_dict(self) -> Dict[str, bool]:
        return asdict(self)

@dataclass
class HistoryPoint:
    """Single trend point for the chart area."""
    ts: float
    o2: Optional[float]
    flow: Optional[float]
    pressure: Optional[float]
    ppo2: Optional[float]
    def to_dict(self) -> Dict[str, Optional[float]]:
        return asdict(self)

@dataclass
class UiState:
    """High-level state returned to the frontend."""
    mode: str = "auto"
    auto_running: bool = True
    auto_path: str = "purge"
    target_o2: float = 1.0
    valves: ValveState = field(default_factory=ValveState)
    estop: bool = False
    fault: bool = False
    fault_message: str = ""
    connected: bool = False
    locked_controls: bool = False
    dimmed: bool = False
    timestamp_str: str = "--"
    last_seen_str: str = "--"
    system_status: str = "System Normal ↗"
    metrics: TelemetryData = field(default_factory=TelemetryData)
    history: List[HistoryPoint] = field(default_factory=list)
    console_text: str = ""
    csv_url: str = ""
    sensor_backend: str = "sim"

    def to_api_dict(self, range_sec: int = 1200) -> Dict[str, object]:
        now = time.time()
        cutoff = now - range_sec
        recent = [p for p in self.history if p.ts >= cutoff]
        
        if len(recent) > 1000:
            step = len(recent) // 500
            recent = recent[::step]
            
        return {
            "mode": self.mode,
            "auto_running": self.auto_running,
            "auto_path": self.auto_path,
            "target_o2": self.target_o2,
            "valves": self.valves.to_dict(),
            "estop": self.estop,
            "fault": self.fault,
            "fault_message": self.fault_message,
            "connected": self.connected,
            "locked_controls": self.locked_controls,
            "dimmed": self.dimmed,
            "timestamp_str": self.timestamp_str,
            "last_seen_str": self.last_seen_str,
            "system_status": self.system_status,
            "metrics": self.metrics.to_dict(),
            "history": [point.to_dict() for point in recent],
            "console_text": self.console_text,
            "csv_url": self.csv_url,
            "sensor_backend": self.sensor_backend,
        }
