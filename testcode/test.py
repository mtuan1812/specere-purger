#!/usr/bin/env python3
"""
Sensor validation script — RPi4
  - LuminOx LOX (UART5, /dev/ttyAMA5) — O2 ppO2, O2%, temp via ASCII stream
  - SFM4300 (I2C 0x2A) — N2 flow rate in slm
  - SHT45  (I2C 0x44) — ambient temp + humidity
Run: python3 sensor_test.py
"""

import time
import struct
import serial
import smbus2
import sys


# ── Thresholds for "sanity check" alarms ──────────────────────────────────────
ALARM_O2_MAX_PCT     = 25.0    # above this in a N2 purge context = sensor likely in air, not chamber
ALARM_FLOW_MAX_SLM   = 20.0    # above this = plumbing issue
ALARM_TEMP_MAX_C     = 60.0    # above this = something's hot
ALARM_RH_MAX_PCT     = 90.0    # above this = condensation risk

_fail_counts = {"lox": 0, "sfm": 0, "sht": 0}

# ── Config ────────────────────────────────────────────────────────────────────
UART_PORT    = "/dev/ttyAMA3"
UART_BAUD    = 9600
I2C_BUS      = 1

SFM4300_ADDR = 0x2A   # ADDR pin floating or GND
SHT45_ADDR   = 0x44

POLL_HZ      = 1      # terminal print rate

# ── CRC-8 (shared by both Sensirion sensors) ──────────────────────────────────
def sensirion_crc(data: bytes) -> int:
    """CRC-8, poly=0x31, init=0xFF — used by SFM4300 and SHT45."""
    crc = 0xFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc

def check_crc(data: bytes, received_crc: int) -> bool:
    return sensirion_crc(data) == received_crc

# ══════════════════════════════════════════════════════════════════════════════
# SFM4300 — Flow Sensor
# ══════════════════════════════════════════════════════════════════════════════
# Scale factor and offset for N2 (use Air calibration — closest available for N2)
# Per datasheet Table 15: scale=2500 slm⁻¹, offset=-28672 for SFM4300-20
# N2 is not a named calibration gas; Air (0x3608) is the best fit.
SFM4300_CMD_START_AIR  = [0x36, 0x08]   # start continuous measurement, Air cal
SFM4300_CMD_STOP       = [0x3F, 0xF9]
SFM4300_SCALE          = 2500.0
SFM4300_OFFSET         = -28672.0
SFM4300_WARMUP_S       = 0.05    # datasheet: first reading ready ~12ms; 50ms is safe
SFM4300_POLL_S         = 0.10    # inter-read delay when sensor is running

_sfm_started = False

def sfm4300_start(bus: smbus2.SMBus):
    global _sfm_started
    if not _sfm_started:
        bus.write_i2c_block_data(SFM4300_ADDR, SFM4300_CMD_START_AIR[0],
                                 SFM4300_CMD_START_AIR[1:])
        time.sleep(SFM4300_WARMUP_S)   # wait for first measurement to be ready
        _sfm_started = True

def sfm4300_read(bus: smbus2.SMBus) -> dict:
    """
    Read 9 bytes: [flow_hi, flow_lo, crc, temp_hi, temp_lo, crc, sw_hi, sw_lo, crc]
    Returns dict with flow_slm, temp_c, status_word, or error string.
    """
    try:
        sfm4300_start(bus)
        time.sleep(SFM4300_POLL_S)   # fixed-period polling
        
        # Raw I2C read (no register address byte sent)
        msg = smbus2.i2c_msg.read(SFM4300_ADDR, 9)
        bus.i2c_rdwr(msg)
        raw = list(msg)

        # flow
        if not check_crc(bytes(raw[0:2]), raw[2]):
            return {"error": "flow CRC fail"}
        flow_raw = struct.unpack(">h", bytes(raw[0:2]))[0]   # signed 16-bit
        flow_slm = (flow_raw - SFM4300_OFFSET) / SFM4300_SCALE

        # temperature
        if not check_crc(bytes(raw[3:5]), raw[5]):
            return {"error": "temp CRC fail"}
        temp_raw = struct.unpack(">h", bytes(raw[3:5]))[0]
        temp_c   = temp_raw / 200.0   # offset=0, scale=200

        # status word (informational)
        if not check_crc(bytes(raw[6:8]), raw[8]):
            return {"error": "status CRC fail"}
        status = (raw[6] << 8) | raw[7]

        return {"flow_slm": flow_slm, "temp_c": temp_c, "status": hex(status)}

    except Exception as e:
        return {"error": str(e)}

# ══════════════════════════════════════════════════════════════════════════════
# SHT45 — Humidity + Temperature
# ══════════════════════════════════════════════════════════════════════════════
SHT45_CMD_MEASURE_HIGH = 0xFD   # high repeatability, ~8.3ms

def sht45_read(bus: smbus2.SMBus) -> dict:
    """
    Send measure command, wait, read 6 bytes:
    [t_hi, t_lo, t_crc, rh_hi, rh_lo, rh_crc]
    """
    try:
        bus.write_byte(SHT45_ADDR, SHT45_CMD_MEASURE_HIGH)
        time.sleep(0.01)   # 8.3ms max measurement time

        # Raw I2C read (no register address byte sent)
        msg = smbus2.i2c_msg.read(SHT45_ADDR, 6)
        bus.i2c_rdwr(msg)
        raw = list(msg)

        if not check_crc(bytes(raw[0:2]), raw[2]):
            return {"error": "temp CRC fail"}
        if not check_crc(bytes(raw[3:5]), raw[5]):
            return {"error": "RH CRC fail"}

        t_ticks  = (raw[0] << 8) | raw[1]
        rh_ticks = (raw[3] << 8) | raw[4]

        temp_c = -45.0 + 175.0 * t_ticks / 65535.0
        rh_pct = max(0.0, min(100.0, -6.0 + 125.0 * rh_ticks / 65535.0))

        return {"temp_c": temp_c, "rh_pct": rh_pct}

    except Exception as e:
        return {"error": str(e)}

# ══════════════════════════════════════════════════════════════════════════════
# LuminOx LOX — O2 sensor (UART5, ASCII stream)
# ══════════════════════════════════════════════════════════════════════════════
# Stream format (default on power-up, ~1Hz):
# "O xxxx.x T yxx.x P xxxx % xxx.xx e xxxx\r\n"
# Poll mode: send "M 1\r\n" to switch, then individual commands like "O\r\n"
# We'll use stream mode — just read lines.

def luminox_parse_stream(line: str) -> dict:
    """
    Parse a full stream line from LuminOx.
    e.g. "O 0210.3 T +20.5 P 1013 % 020.90 e 0000"
    Returns dict with ppo2_mbar, temp_c, pressure_mbar, o2_pct, status
    """
    result = {}
    try:
        parts = line.strip().split()
        # parts: ['O','0210.3','T','+20.5','P','1013','%','020.90','e','0000']
        it = iter(parts)
        for token in it:
            val = next(it, None)
            if val is None:
                break
            if token == "O":
                result["ppo2_mbar"] = float(val)
            elif token == "T":
                result["temp_c"] = float(val)
            elif token == "P":
                result["pressure_mbar"] = float(val)
            elif token == "%":
                result["o2_pct"] = float(val)
            elif token == "e":
                result["status"] = val
                result["status_ok"] = (val == "0000")
    except Exception as e:
        result["error"] = str(e)
    return result

def luminox_read_line(ser: serial.Serial) -> dict:
    """Read one complete line from LuminOx. Returns parsed dict or error."""
    try:
        raw = ser.readline().decode("ascii", errors="replace").strip()
        if not raw:
            return {"error": "timeout / no data"}
        if raw.startswith("E "):
            return {"error": f"sensor error: {raw}"}
        return luminox_parse_stream(raw)
    except Exception as e:
        return {"error": str(e)}

# ── Pretty printer ────────────────────────────────────────────────────────────
def fmt(val, fmt_str, unit, err="ERR"):
    if val is None:
        return err
    try:
        return f"{val:{fmt_str}}{unit}"
    except Exception:
        return err

def print_readings(lox: dict, sfm: dict, sht: dict):
    sep = "─" * 72

    # LuminOx
    if "error" in lox:
        lox_line = f"  O2: ERR ({lox['error']})"
    else:
        status_flag = "✓" if lox.get("status_ok") else f"! {lox.get('status')}"
        lox_line = (
            f"  O2:    {lox.get('o2_pct', '---'):>7.2f} %   "
            f"ppO2: {lox.get('ppo2_mbar', '---'):>7.1f} mbar   "
            f"T: {lox.get('temp_c', '---'):>6.1f} °C   [{status_flag}]"
        )

    # SFM4300
    if "error" in sfm:
        sfm_line = f"  Flow: ERR ({sfm['error']})"
    else:
        sfm_line = (
            f"  Flow:  {sfm.get('flow_slm', '---'):>7.3f} slm  "
            f"T(FM): {sfm.get('temp_c', '---'):>6.1f} °C   "
            f"status: {sfm.get('status', '?')}"
        )

    # SHT45
    if "error" in sht:
        sht_line = f"  Amb:  ERR ({sht['error']})"
    else:
        sht_line = (
            f"  Amb:   T: {sht.get('temp_c', '---'):>6.2f} °C   "
            f"RH: {sht.get('rh_pct', '---'):>6.2f} %"
        )

    print(sep)
    print(lox_line)
    print(sfm_line)
    print(sht_line)

def check_and_alarm(lox: dict, sfm: dict, sht: dict):
    alarms = []

    # ── Sensor communication failures ────────────────────────────────────────
    for name, reading in [("lox", lox), ("sfm", sfm), ("sht", sht)]:
        if "error" in reading:
            _fail_counts[name] += 1
            alarms.append(
                f"[COMM FAIL x{_fail_counts[name]}] {name.upper()}: {reading['error']}"
            )
        else:
            _fail_counts[name] = 0   # reset on success

    # ── LuminOx internal status flag ─────────────────────────────────────────
    if "status_ok" in lox and not lox["status_ok"]:
        alarms.append(f"[SENSOR FAULT] LuminOx status: {lox.get('status')}")

    # ── Value sanity checks ───────────────────────────────────────────────────
    if "o2_pct" in lox and lox["o2_pct"] > ALARM_O2_MAX_PCT:
        alarms.append(f"[RANGE] O2 {lox['o2_pct']:.2f}% > {ALARM_O2_MAX_PCT}%")

    if "flow_slm" in sfm and sfm["flow_slm"] > ALARM_FLOW_MAX_SLM:
        alarms.append(f"[RANGE] Flow {sfm['flow_slm']:.3f} slm > {ALARM_FLOW_MAX_SLM}")

    if "temp_c" in sht and sht["temp_c"] > ALARM_TEMP_MAX_C:
        alarms.append(f"[RANGE] Ambient temp {sht['temp_c']:.1f}°C > {ALARM_TEMP_MAX_C}°C")

    if "rh_pct" in sht and sht["rh_pct"] > ALARM_RH_MAX_PCT:
        alarms.append(f"[RANGE] RH {sht['rh_pct']:.1f}% > {ALARM_RH_MAX_PCT}%")

    # ── Print alarms loud ─────────────────────────────────────────────────────
    for alarm in alarms:
        print(f"\033[91m  *** ALARM: {alarm} ***\033[0m", file=sys.stderr)

    return len(alarms) == 0   # True = all clear

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    bus = smbus2.SMBus(I2C_BUS)
    ser = serial.Serial(
        port=UART_PORT,
        baudrate=UART_BAUD,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1,
    )

    ser.write(b"M 0\r\n")
    time.sleep(0.5)
    ser.reset_input_buffer()

    print("Sensor test — Ctrl+C to quit")

    try:
        while True:
            lox = luminox_read_line(ser)
            sfm = sfm4300_read(bus)    # now IRQ-driven
            sht = sht45_read(bus)
            print_readings(lox, sfm, sht)
            check_and_alarm(lox, sfm, sht)
            time.sleep(1 / POLL_HZ)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        try:
            bus.write_i2c_block_data(SFM4300_ADDR, SFM4300_CMD_STOP[0],
                                     SFM4300_CMD_STOP[1:])
        except Exception:
            pass
        bus.close()
        ser.close()

if __name__ == "__main__":
    main()
