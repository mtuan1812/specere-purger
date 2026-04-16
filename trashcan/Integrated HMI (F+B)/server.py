#!/usr/bin/env python3
import csv, importlib.util, io, json, math, os, threading, time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse, unquote
from PIL import Image, ImageDraw
BASE_DIR=os.path.dirname(os.path.abspath(__file__));CSV_PATH=os.path.join(BASE_DIR,"telemetry.csv");TEST_PATH=os.path.join(BASE_DIR,"test.py")
def load_test_module():
    try:
        spec=importlib.util.spec_from_file_location("sensor_test_module",TEST_PATH)
        if spec is None or spec.loader is None: raise RuntimeError("Could not create module spec for test.py")
        mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(mod); return mod,None
    except Exception as e: return None,e
test_mod,test_mod_error=load_test_module()
class RuntimeState:
    def __init__(self):
        self.lock=threading.Lock(); self.mode="auto"; self.auto_running=True; self.auto_path="purge"; self.target_o2=1.0
        self.valves={"purge":False,"steady":False}; self.estop=False; self.fault=False; self.fault_message=""; self.connected=False
        self.console_lines=[]; self.dimmed=False; self.locked_controls=False; self.history=[]
        self.latest_metrics={"o2_pct":None,"flow_slm":None,"pressure_mbar":None,"ppo2":None,"temp_c":None,"rh_pct":None}
        self.sensor_backend="sim"; self._sensor_objects=None; self._init_sensors(); self._ensure_csv()
    def _ensure_csv(self):
        if not os.path.exists(CSV_PATH):
            with open(CSV_PATH,"w",newline="",encoding="utf-8") as f:
                csv.writer(f).writerow(["timestamp_iso","epoch_s","o2_pct","flow_slm","pressure_mbar","ppo2_mbar","temp_c","rh_pct","purge_valve","steady_valve","mode","auto_running","fault","estop"])
    def log(self,text):
        stamp=datetime.now().strftime("%m/%d/%y %I:%M:%S %p"); self.console_lines.append(f"[{stamp}] {text}"); self.console_lines=self.console_lines[-200:]
    def _init_sensors(self):
        if test_mod is None:
            self._sensor_objects=None; self.connected=False; self.sensor_backend="sim"; self.log(f"test.py unavailable on this system; using simulation ({test_mod_error})"); return
        try:
            bus=test_mod.smbus2.SMBus(test_mod.I2C_BUS)
            ser=test_mod.serial.Serial(port=test_mod.UART_PORT,baudrate=test_mod.UART_BAUD,bytesize=test_mod.serial.EIGHTBITS,parity=test_mod.serial.PARITY_NONE,stopbits=test_mod.serial.STOPBITS_ONE,timeout=1)
            ser.write(b"M 0\r\n"); time.sleep(.5); ser.reset_input_buffer()
            self._sensor_objects={"bus":bus,"ser":ser}; self.connected=True; self.sensor_backend="test.py"; self.log("Sensor backend initialized from test.py")
        except Exception as e:
            self._sensor_objects=None; self.connected=False; self.sensor_backend="sim"; self.log(f"Sensor backend unavailable; using simulation ({e})")
    def _close_sensors(self):
        if not self._sensor_objects: return
        try:
            bus=self._sensor_objects["bus"]
            try:
                if test_mod is not None: bus.write_i2c_block_data(test_mod.SFM4300_ADDR,test_mod.SFM4300_CMD_STOP[0],test_mod.SFM4300_CMD_STOP[1:])
            except Exception: pass
            bus.close()
        except Exception: pass
        try: self._sensor_objects["ser"].close()
        except Exception: pass
        self._sensor_objects=None
    def _simulate_metrics(self):
        t=time.time(); o2=max(.1,2.2+.6*math.sin(t/18.0)-(.8 if self.valves["purge"] else 0)-(.3 if self.valves["steady"] else 0)); flow=(5.5 if self.valves["purge"] else 0.0)+(.7 if self.valves["steady"] else 0.0)
        return {"o2_pct":round(o2,2),"flow_slm":round(flow,2),"pressure_mbar":round(1013+4*math.sin(t/30.0),2),"ppo2":round(max(0,o2/100.0*1013),1),"temp_c":round(24+1.0*math.sin(t/40.0),2),"rh_pct":round(48+8*math.sin(t/55.0),2)}
    def read_once(self):
        if self._sensor_objects and test_mod is not None:
            bus=self._sensor_objects["bus"]; ser=self._sensor_objects["ser"]
            try:
                lox=test_mod.luminox_read_line(ser); sfm=test_mod.sfm4300_read(bus); sht=test_mod.sht45_read(bus)
                metrics={"o2_pct":None if "error" in lox else lox.get("o2_pct"),"flow_slm":None if "error" in sfm else sfm.get("flow_slm"),"pressure_mbar":None if "error" in lox else lox.get("pressure_mbar"),"ppo2":None if "error" in lox else lox.get("ppo2_mbar"),"temp_c":None if "error" in sht else sht.get("temp_c"),"rh_pct":None if "error" in sht else sht.get("rh_pct")}
                faults=[]; 
                if "error" in lox: faults.append(f"LOX: {lox['error']}")
                if "error" in sfm: faults.append(f"SFM4300: {sfm['error']}")
                if "error" in sht: faults.append(f"SHT45: {sht['error']}")
                if "status_ok" in lox and not lox.get("status_ok",False): faults.append(f"LuminOx status {lox.get('status')}")
                status_flag="✓" if lox.get("status_ok") else (f"! {lox.get('status')}" if "status" in lox else "?")
                self.log(f"O2: {metrics['o2_pct'] if metrics['o2_pct'] is not None else 'ERR'} %   ppO2: {metrics['ppo2'] if metrics['ppo2'] is not None else 'ERR'} mbar   P: {metrics['pressure_mbar'] if metrics['pressure_mbar'] is not None else 'ERR'} mbar   [{status_flag}]")
                self.log(f"Flow: {metrics['flow_slm'] if metrics['flow_slm'] is not None else 'ERR'} slm")
                self.log(f"Amb: T {metrics['temp_c'] if metrics['temp_c'] is not None else 'ERR'} C   RH {metrics['rh_pct'] if metrics['rh_pct'] is not None else 'ERR'} %")
                return metrics,faults
            except Exception as e:
                self.connected=False; self.log(f"Sensor read failure; switching to simulation ({e})"); self._close_sensors(); self.sensor_backend="sim"
        return self._simulate_metrics(),[]
    def step(self):
        metrics,fault_msgs=self.read_once(); now=time.time()
        with self.lock:
            self.latest_metrics=metrics; self.connected=(self.sensor_backend=="test.py")
            if self.estop:
                self.valves["purge"]=False; self.valves["steady"]=False; self.fault=True; self.fault_message="E-stop active"
            else:
                if self.mode=="auto" and self.auto_running:
                    o2=metrics.get("o2_pct")
                    if o2 is not None and o2>self.target_o2:
                        self.valves["purge"]=(self.auto_path=="purge"); self.valves["steady"]=(self.auto_path=="steady")
                    else:
                        self.valves["purge"]=False; self.valves["steady"]=False
                if fault_msgs: self.fault=True; self.fault_message="; ".join(fault_msgs)
                elif not self.estop: self.fault=False; self.fault_message=""
            self.history.append({"ts":now,"o2":metrics.get("o2_pct"),"flow":metrics.get("flow_slm"),"pressure":metrics.get("pressure_mbar"),"ppo2":metrics.get("ppo2")}); self.history=self.history[-90000:]
            with open(CSV_PATH,"a",newline="",encoding="utf-8") as f:
                csv.writer(f).writerow([datetime.fromtimestamp(now).isoformat(sep=" ",timespec="seconds"),int(now),metrics.get("o2_pct"),metrics.get("flow_slm"),metrics.get("pressure_mbar"),metrics.get("ppo2"),metrics.get("temp_c"),metrics.get("rh_pct"),int(self.valves["purge"]),int(self.valves["steady"]),self.mode,int(self.auto_running),int(self.fault),int(self.estop)])
    def command(self,action,data):
        with self.lock:
            if action=="set_mode": self.mode="auto" if data.get("mode")=="auto" else "manual"
            elif action=="adjust_setpoint": self.target_o2=min(25.0,max(.1,round(self.target_o2+float(data.get("delta",0)),1)))
            elif action=="toggle_auto_running":
                self.auto_running=not self.auto_running
                if not self.auto_running: self.valves["purge"]=False; self.valves["steady"]=False
            elif action=="set_auto_path": self.auto_path="steady" if data.get("path")=="steady" else "purge"
            elif action=="toggle_valve":
                if not self.locked_controls and not self.estop:
                    v=data.get("valve")
                    if v=="purge": self.valves["purge"]=not self.valves["purge"]
                    elif v=="steady": self.valves["steady"]=not self.valves["steady"]
            elif action=="toggle_estop":
                self.estop=not self.estop
                if self.estop: self.valves["purge"]=False; self.valves["steady"]=False; self.log("E-stop activated")
                else: self.log("E-stop cleared")
            elif action=="reset_faults":
                if not self.estop: self.fault=False; self.fault_message=""
                self.log("Fault reset requested")
            elif action=="toggle_dim": self.dimmed=not self.dimmed
            elif action=="toggle_lock": self.locked_controls=not self.locked_controls
        return self.snapshot("localhost:8000")
    def snapshot(self,host):
        with self.lock:
            try: ts=datetime.now().strftime("%-m/%-d/%y %-I:%M:%S %p")
            except ValueError: ts=datetime.now().strftime("%m/%d/%y %I:%M:%S %p").lstrip("0").replace("/0","/")
            status_text="System Normal ↗"
            if self.estop: status_text="E-Stop Active ↗"
            elif self.fault: status_text="System Fault ↗"
            return {"mode":self.mode,"auto_running":self.auto_running,"auto_path":self.auto_path,"target_o2":self.target_o2,"valves":dict(self.valves),"estop":self.estop,"fault":self.fault,"fault_message":self.fault_message,"connected":self.connected,"locked_controls":self.locked_controls,"dimmed":self.dimmed,"timestamp_str":ts,"system_status":status_text,"metrics":dict(self.latest_metrics),"history":list(self.history),"console_text":"\n".join(self.console_lines[-120:]),"csv_url":f"http://{host}/telemetry.csv","sensor_backend":self.sensor_backend}
runtime=RuntimeState()
class PollThread(threading.Thread):
    daemon=True
    def run(self):
        while True:
            try: runtime.step()
            except Exception as e: runtime.log(f"Background loop error: {e}")
            time.sleep(1.0)
class Handler(BaseHTTPRequestHandler):
    def _send(self,code,data,content_type="text/plain; charset=utf-8"):
        self.send_response(code); self.send_header("Content-Type",content_type); self.send_header("Cache-Control","no-store"); self.end_headers()
        if isinstance(data,str): data=data.encode("utf-8")
        self.wfile.write(data)
    def log_message(self,fmt,*args): pass
    def do_GET(self):
        parsed=urlparse(self.path); path=parsed.path
        if path=="/api/state":
            host=self.headers.get("Host","localhost:8000"); self._send(200,json.dumps(runtime.snapshot(host)),"application/json"); return
        if path=="/telemetry.csv":
            with open(CSV_PATH,"rb") as f: self._send(200,f.read(),"text/csv; charset=utf-8"); return
        if path=="/api/qr.png":
            qs=parse_qs(parsed.query); url=unquote(qs.get("url",["http://localhost:8000/telemetry.csv"])[0]); self._send(200,self.render_url_card(url),"image/png"); return
        if path=="/api/export_plot":
            qs=parse_qs(parsed.query); metric=qs.get("metric",["flow"])[0]; range_sec=int(qs.get("range",["1200"])[0]); self._send(200,self.render_plot(metric,range_sec),"image/png"); return
        if path=="/": path="/index.html"
        local=os.path.join(BASE_DIR,path.lstrip("/"))
        if os.path.isfile(local):
            ctype="text/plain; charset=utf-8"
            if local.endswith(".html"): ctype="text/html; charset=utf-8"
            elif local.endswith(".css"): ctype="text/css; charset=utf-8"
            elif local.endswith(".js"): ctype="application/javascript; charset=utf-8"
            elif local.endswith(".png"): ctype="image/png"
            with open(local,"rb") as f: self._send(200,f.read(),ctype); return
        self._send(404,"Not found")
    def render_url_card(self,url):
        width,height=520,520; img=Image.new("RGB",(width,height),"white"); d=ImageDraw.Draw(img)
        d.rounded_rectangle([12,12,width-12,height-12],radius=18,outline="black",width=3); d.text((30,28),"Local CSV URL",fill="black")
        qr_x,qr_y,qr_size=110,80,300; d.rectangle([qr_x,qr_y,qr_x+qr_size,qr_y+qr_size],outline="black",width=3)
        cell=12; seed=sum(ord(c) for c in url)
        for r in range(21):
            for c in range(21):
                v=(seed+r*37+c*19+(r*c))%7
                if v in (0,1,3):
                    x1=qr_x+20+c*cell; y1=qr_y+20+r*cell; x2=x1+cell-2; y2=y1+cell-2
                    if x2<qr_x+qr_size-20 and y2<qr_y+qr_size-20: d.rectangle([x1,y1,x2,y2],fill="black")
        wrapped=[]; line=""
        for part in url.split("/"):
            cand=part if not line else line+"/"+part
            if len(cand)>34:
                if line: wrapped.append(line)
                line=part
            else: line=cand
        if line: wrapped.append(line)
        y=405
        for ln in wrapped[:4]:
            d.text((28,y),ln,fill="black"); y+=24
        buf=io.BytesIO(); img.save(buf,format="PNG"); return buf.getvalue()
    def render_plot(self,metric,range_sec):
        width,height=800,480; img=Image.new("RGB",(width,height),(243,243,243)); d=ImageDraw.Draw(img); left,top,right,bottom=54,24,width-18,height-46
        d.rectangle([left,top,right,bottom],outline=(200,200,200))
        for i in range(6):
            y=top+(bottom-top)*i/5; d.line([left,y,right,y],fill=(196,196,196),width=1)
        for i in range(7):
            x=left+(right-left)*i/6; d.line([x,top,x,bottom],fill=(196,196,196),width=1)
        hist=[h for h in runtime.history if h["ts"]>=time.time()-range_sec and h.get(metric) is not None]
        if len(hist)>=2:
            vals=[float(h[metric]) for h in hist]; lo,hi=min(vals),max(vals); span=max(hi-lo,.001); lo-=span*.1; hi+=span*.1; t0,t1=hist[0]["ts"],hist[-1]["ts"]; tspan=max(t1-t0,1); pts=[]
            for h in hist:
                x=left+(h["ts"]-t0)/tspan*(right-left); y=top+(1-((float(h[metric])-lo)/(hi-lo)))*(bottom-top); pts.append((x,y))
            d.line(pts,fill=(42,161,152),width=3)
        buf=io.BytesIO(); img.save(buf,format="PNG"); return buf.getvalue()
    def do_POST(self):
        parsed=urlparse(self.path)
        if parsed.path!="/api/command": self._send(404,"Not found"); return
        length=int(self.headers.get("Content-Length","0")); raw=self.rfile.read(length).decode("utf-8") if length else "{}"; data=json.loads(raw or "{}"); action=data.get("action",""); host=self.headers.get("Host","localhost:8000")
        snap=runtime.command(action,data); snap["csv_url"]=f"http://{host}/telemetry.csv"; self._send(200,json.dumps(snap),"application/json")
if __name__=="__main__":
    PollThread().start(); server=ThreadingHTTPServer(("0.0.0.0",8000),Handler); print("Serving on http://0.0.0.0:8000"); 
    try: server.serve_forever()
    except KeyboardInterrupt: pass
