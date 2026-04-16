
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
                li_elements = []
                for f in files:
                    try: size_kb = os.path.getsize(os.path.join(tel_dir, f)) / 1024
                    except OSError: size_kb = 0
                    li_elements.append(f"<li><a href='/telemetry/{f}'><span>📄 {f}</span><span style='color:#777; font-size:14px; font-weight:normal;'>{size_kb:.1f} KB</span></a></li>")
                if not files: 
                    li_elements.append("<li style='padding:20px; color:#666; text-align:center;'>No logs found.</li>")
                
                template_path = os.path.join(BASE_DIR, "telemetry_index.html")
                try:
                    with open(template_path, "r", encoding="utf-8") as f:
                        template = f.read()
                except FileNotFoundError:
                    template = "<html><body><ul>{{FILE_LIST}}</ul></body></html>"
                
                html = template.replace("{{FILE_LIST}}", "".join(li_elements))
                self._send(200, html.encode('utf-8'), "text/html; charset=utf-8"); return
            if path == "/": path = "/index.html"
            local = os.path.join(BASE_DIR, path.lstrip("/"))
            if os.path.isfile(local):
                ctype = "text/plain; charset=utf-8"
                if local.endswith(".html"): ctype = "text/html; charset=utf-8"
                elif local.endswith(".css"): ctype = "text/css; charset=utf-8"
                elif local.endswith(".js"): ctype = "application/javascript; charset=utf-8"
                elif local.endswith(".png"): ctype = "image/png"
                elif local.endswith(".csv"): ctype = "text/csv; charset=utf-8"
                elif local.endswith(".ttf"): ctype = "font/ttf"
                elif local.endswith(".woff"): ctype = "font/woff"
                elif local.endswith(".woff2"): ctype = "font/woff2"
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
            import qrcode
            qr = qrcode.QRCode(version=1, error_correction=qrcode.constants.ERROR_CORRECT_L, box_size=10, border=2)
            qr.add_data(url)
            qr.make(fit=True)
            qr_wrapper = qr.make_image(fill_color="black", back_color="white")
            qr_img = getattr(qr_wrapper, "get_image", lambda: qr_wrapper)().convert("RGB")
            
            width, height = 520, 520
            img = Image.new("RGB", (width, height), "white")
            draw = ImageDraw.Draw(img)
            draw.rounded_rectangle([12, 12, width - 12, height - 12], radius=18, outline="black", width=3)
            
            # Center the QR
            qr_w, qr_h = qr_img.size
            if qr_w > 400: qr_img = qr_img.resize((400, 400), Image.Resampling.NEAREST)
            qr_w, qr_h = qr_img.size
            
            x_offset = (width - qr_w) // 2
            y_offset = (height - qr_h) // 2 - 20
            img.paste(qr_img, (x_offset, y_offset))
            
            # Draw URL perfectly centered at bottom
            text_y = y_offset + qr_h + 30
            # Simple centering logic
            url_len = len(url)
            char_w = 7 # approximate monospace
            x_text = max(20, (width - url_len * char_w) // 2)
            draw.text((x_text, text_y), url, fill="#444")
            
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
