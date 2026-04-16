
import io, json, os, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse, unquote
from PIL import Image, ImageDraw

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

class PollThread(threading.Thread):
    daemon = True
    def __init__(self, runtime):
        super().__init__()
        self.runtime = runtime
    def run(self):
        while True:
            try: self.runtime.step()
            except Exception as exc: self.runtime.log(f"Background loop error: {exc}")
            time.sleep(1.0)

def make_handler(runtime):
    class Handler(BaseHTTPRequestHandler):
        def _send(self, code, data, content_type="text/plain; charset=utf-8"):
            self.send_response(code); self.send_header("Content-Type", content_type); self.send_header("Cache-Control", "no-store"); self.end_headers()
            if isinstance(data, str): data = data.encode("utf-8")
            self.wfile.write(data)
        def log_message(self, fmt, *args): pass

        def do_GET(self):
            parsed = urlparse(self.path); path = parsed.path
            if path == "/api/state":
                host = self.headers.get("Host", "localhost:8000")
                self._send(200, json.dumps(runtime.snapshot_dict(host)), "application/json"); return
            if path == "/api/qr.png":
                qs = parse_qs(parsed.query); url = unquote(qs.get("url", ["http://localhost:8000/telemetry.csv"])[0])
                self._send(200, self.render_url_card(url), "image/png"); return
            if path == "/api/export_plot":
                qs = parse_qs(parsed.query); metric = qs.get("metric", ["flow"])[0]; range_sec = int(qs.get("range", ["1200"])[0])
                self._send(200, self.render_plot(metric, range_sec), "image/png"); return
            if path in ("/telemetry", "/telemetry/"):
                tel_dir = os.path.join(BASE_DIR, "telemetry")
                os.makedirs(tel_dir, exist_ok=True)
                files = sorted([f for f in os.listdir(tel_dir) if f.endswith(".csv")], reverse=True)
                html = ["<!DOCTYPE html><html><head><title>Telemetry Logs</title>"]
                html.append("<meta name='viewport' content='width=device-width, initial-scale=1'>")
                html.append("<style>body{font-family:sans-serif; margin:2rem; background:#f4f4f9;} ul{list-style:none; padding:0; background:#fff; border-radius:8px; box-shadow:0 2px 12px rgba(0,0,0,0.1); overflow:hidden;} li{border-bottom:1px solid #eee;} li:last-child{border-bottom:none;} a{display:flex; justify-content:space-between; align-items:center; padding:16px 20px; text-decoration:none; color:#1a73e8; font-size:18px;} a:hover{background:#f8f9fa;}</style>")
                html.append("</head><body><h2 style='color:#333; margin-bottom:20px;'>Telemetry CSV Logs</h2><ul>")
                for f in files:
                    try: size_kb = os.path.getsize(os.path.join(tel_dir, f)) / 1024
                    except OSError: size_kb = 0
                    html.append(f"<li><a href='/telemetry/{f}'><span>📄 {f}</span><span style='color:#777; font-size:14px; font-weight:normal;'>{size_kb:.1f} KB</span></a></li>")
                if not files: html.append("<li style='padding:20px; color:#666; text-align:center;'>No logs found.</li>")
                html.append("</ul></body></html>")
                self._send(200, "".join(html), "text/html; charset=utf-8"); return
            if path == "/": path = "/index.html"
            local = os.path.join(BASE_DIR, path.lstrip("/"))
            if os.path.isfile(local):
                ctype = "text/plain; charset=utf-8"
                if local.endswith(".html"): ctype = "text/html; charset=utf-8"
                elif local.endswith(".css"): ctype = "text/css; charset=utf-8"
                elif local.endswith(".js"): ctype = "application/javascript; charset=utf-8"
                elif local.endswith(".png"): ctype = "image/png"
                elif local.endswith(".csv"): ctype = "text/csv; charset=utf-8"
                with open(local, "rb") as f: self._send(200, f.read(), ctype); return
            self._send(404, "Not found")

        def do_POST(self):
            parsed = urlparse(self.path)
            if parsed.path != "/api/command":
                self._send(404, "Not found"); return
            length = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(length).decode("utf-8") if length else "{}"
            data = json.loads(raw or "{}")
            runtime.handle_command(data.get("action", ""), data)
            host = self.headers.get("Host", "localhost:8000")
            self._send(200, json.dumps(runtime.snapshot_dict(host)), "application/json")

        def render_url_card(self, url):
            width, height = 520, 520
            img = Image.new("RGB", (width, height), "white")
            draw = ImageDraw.Draw(img)
            draw.rounded_rectangle([12, 12, width - 12, height - 12], radius=18, outline="black", width=3)
            draw.text((30, 28), "Local CSV URL", fill="black")
            qr_x, qr_y, qr_size = 110, 80, 300
            draw.rectangle([qr_x, qr_y, qr_x + qr_size, qr_y + qr_size], outline="black", width=3)
            cell = 12; seed = sum(ord(ch) for ch in url)
            for row in range(21):
                for col in range(21):
                    value = (seed + row * 37 + col * 19 + (row * col)) % 7
                    if value in (0, 1, 3):
                        x1 = qr_x + 20 + col * cell; y1 = qr_y + 20 + row * cell; x2 = x1 + cell - 2; y2 = y1 + cell - 2
                        if x2 < qr_x + qr_size - 20 and y2 < qr_y + qr_size - 20: draw.rectangle([x1, y1, x2, y2], fill="black")
            wrapped=[]; line=""
            for part in url.split("/"):
                candidate = part if not line else line + "/" + part
                if len(candidate) > 34:
                    if line: wrapped.append(line)
                    line = part
                else: line = candidate
            if line: wrapped.append(line)
            y = 405
            for line in wrapped[:4]:
                draw.text((28, y), line, fill="black"); y += 24
            buffer = io.BytesIO(); img.save(buffer, format="PNG"); return buffer.getvalue()

        def render_plot(self, metric, range_sec):
            width, height = 800, 480
            img = Image.new("RGB", (width, height), (243,243,243))
            draw = ImageDraw.Draw(img)
            left, top, right, bottom = 54, 24, width - 18, height - 46
            draw.rectangle([left, top, right, bottom], outline=(200,200,200))
            for i in range(6):
                y = top + (bottom-top) * i / 5; draw.line([left,y,right,y], fill=(196,196,196), width=1)
            for i in range(7):
                x = left + (right-left) * i / 6; draw.line([x,top,x,bottom], fill=(196,196,196), width=1)
            history = [point for point in runtime.state.history if point.ts >= time.time() - range_sec and getattr(point, metric, None) is not None]
            if len(history) >= 2:
                values = [float(getattr(point, metric)) for point in history]
                low, high = min(values), max(values); span = max(high - low, 0.001); low -= span * 0.1; high += span * 0.1
                first_ts, last_ts = history[0].ts, history[-1].ts; time_span = max(last_ts - first_ts, 1)
                points=[]
                for point in history:
                    x = left + (point.ts - first_ts) / time_span * (right-left)
                    y = top + (1 - ((float(getattr(point, metric)) - low) / (high - low))) * (bottom-top)
                    points.append((x, y))
                draw.line(points, fill=(42,161,152), width=3)
            buffer = io.BytesIO(); img.save(buffer, format="PNG"); return buffer.getvalue()
    return Handler

def create_server(runtime):
    handler = make_handler(runtime)
    poller = PollThread(runtime)
    server = ThreadingHTTPServer(("0.0.0.0", 8000), handler)
    return server, poller
