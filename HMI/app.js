const STATE_URL = "/api/state", COMMAND_URL = "/api/command", EXPORT_URL = "/api/export_plot";
const state = { mode: "auto", metric: "flow", rangeSec: 1200, latest: null };
const $ = id => document.getElementById(id);

function post(action, payload = {}) {
    return fetch(COMMAND_URL, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ action, ...payload })
    }).then(r => r.json());
}

function fmt(n, dp = 2) {
    return (n === null || n === undefined || Number.isNaN(Number(n))) ? "--" : Number(n).toFixed(dp);
}

function setMode(mode) {
    state.mode = mode;
    const isAutoTab = mode === "auto" || mode === "standby";
    $("tabAuto").classList.toggle("active", isAutoTab);
    $("tabManual").classList.toggle("active", mode === "manual");
    $("autoArea").classList.toggle("hidden", !isAutoTab);
    $("manualArea").classList.toggle("hidden", mode !== "manual");
}

function applyPathUI(valves) {
    $("pathPurge").className = "pathBtn" + (valves.dio0 ? " active-purge" : "");
    $("pathSteady").className = "pathBtn" + (valves.dio1 ? " active-steady" : "");
}

function applyStartStop(running) {
    $("startStopBtn").textContent = running ? "STOP" : "START";
    $("startStopBtn").classList.toggle("is-start", !running);
}

function applyStatusBar(fault, estop, ts, text, lastSeen) {
    const red = !!fault || !!estop;
    $("statusBar").classList.toggle("fault", red);
    $("statusTime").textContent = ts || "--";
    $("statusText").textContent = text || (red ? "System Fault ↗" : "System Normal ↗");
    $("lastSeenText").textContent = `Data last seen: ${lastSeen || "--"}`;
}

function updateDio(v, estop, fault) {
    const isFault = estop || fault;
    const getClass = (isOn) => isFault ? "dio fault" : (isOn ? "dio on" : "dio");
    $("dio0").className = getClass(v.dio0);
    $("dio1").className = getClass(v.dio1);
    $("dio2").className = getClass(v.dio2);
    $("dio3").className = getClass(v.dio3);
}

function updateReadings(m) {
    $("valO2").textContent = `${fmt(m.o2_pct, 2)} %`;
    $("valFlow").textContent = `${fmt(m.flow_slm, 2)} slm`;
    $("valPressure").textContent = `${fmt(m.pressure_mbar, 2)} mbar`;
    $("valTemp").textContent = `${fmt(m.temp_c, 2)} °C`;
    $("valRh").textContent = `${fmt(m.rh_pct, 2)} %`;
}

function openConsole(t) {
    const el = $("consoleText");
    el.textContent = t || "";
    $("consoleModal").classList.remove("hidden");
    el.scrollTop = el.scrollHeight;
}

function closeConsole() {
    $("consoleModal").classList.add("hidden");
}

function openQr(u) {
    $("csvUrlText").textContent = u;
    $("qrImage").src = `/api/qr.png?url=${encodeURIComponent(u)}`;
    $("qrModal").classList.remove("hidden");
}

function closeQr() {
    $("qrModal").classList.add("hidden");
}

function drawChart(history) {
    const canvas = $("chartCanvas"), parent = canvas.parentElement;
    canvas.style.width = "0px";
    canvas.style.height = "0px";
    
    const width = parent.clientWidth, height = parent.clientHeight;
    canvas.width = width * 2;
    canvas.height = height * 2;
    canvas.style.width = width + "px";
    canvas.style.height = height + "px";
    
    const ctx = canvas.getContext("2d");
    ctx.scale(2, 2);
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = "#f3f3f3";
    ctx.fillRect(0, 0, width, height);
    
    if (!window.SHOW_PLOT) {
        $("dummyOverlay").classList.remove("hidden");
        return;
    }
    
    $("dummyOverlay").classList.add("hidden");
    const padL = 42, padR = 8, padT = 12, padB = 30;
    const plotW = width - padL - padR, plotH = height - padT - padB;
    
    ctx.strokeStyle = "#c4c4c4";
    ctx.lineWidth = 1;
    
    for (let i = 0; i <= 5; i++) {
        const y = padT + plotH * i / 5;
        ctx.beginPath(); 
        ctx.moveTo(padL, y); 
        ctx.lineTo(padL + plotW, y); 
        ctx.stroke();
    }
    
    for (let i = 0; i <= 6; i++) {
        const x = padL + plotW * i / 6;
        ctx.beginPath(); 
        ctx.moveTo(x, padT); 
        ctx.lineTo(x, padT + plotH); 
        ctx.stroke();
    }
    
    const metric = state.metric, data = history || [];
    if (data.length < 2) {
        ctx.fillStyle = "#333"; 
        ctx.font = "16px Arial";
        ctx.fillText("No data", width / 2 - 25, height / 2);
        return;
    }
    
    const values = data.map(d => Number(d[metric])).filter(v => Number.isFinite(v));
    const minV = Math.min(...values), maxV = Math.max(...values);
    const span = Math.max(maxV - minV, .001), padV = span * .1;
    const lo = minV - padV, hi = maxV + padV;
    
    const firstTs = data[0].ts, lastTs = data[data.length - 1].ts;
    const tSpan = Math.max(lastTs - firstTs, 1);
    
    ctx.strokeStyle = "#2aa198";
    ctx.lineWidth = 2;
    ctx.beginPath();
    
    data.forEach((d, idx) => {
        const x = padL + ((d.ts - firstTs) / tSpan) * plotW;
        const y = padT + (1 - ((Number(d[metric]) - lo) / (hi - lo))) * plotH;
        if (idx === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
    });
    
    ctx.stroke();
    ctx.fillStyle = "#444";
    ctx.font = "12px Arial";
    
    let dp = 0;
    if (span < 5) dp = 2;
    else if (span < 20) dp = 1;
    
    for (let i = 0; i <= 5; i++) {
        const val = hi - (hi - lo) * i / 5;
        const y = padT + plotH * i / 5;
        ctx.fillText(val.toFixed(dp), 4, y + 4);
    }
    
    const labelCount = Math.min(4, data.length);
    for (let i = 0; i < labelCount; i++) {
        const idx = Math.floor(i * (data.length - 1) / (labelCount - 1 || 1));
        const d = data[idx];
        const x = padL + ((d.ts - firstTs) / tSpan) * plotW;
        const dt = new Date(d.ts * 1000);
        const label = dt.toLocaleTimeString([], { hour: "numeric", minute: "2-digit" });
        ctx.save();
        ctx.translate(x, height - 8);
        ctx.rotate(-Math.PI / 2);
        ctx.fillText(label, 0, 0);
        ctx.restore();
    }
}

async function refreshState() {
    const res = await fetch(`${STATE_URL}?range=${state.rangeSec}`, { cache: "no-store" });
    const data = await res.json();
    state.latest = data;
    
    setMode(data.mode);
    applyPathUI(data.valves);
    applyStartStop(data.mode === "auto");
    updateReadings(data.metrics);
    updateDio(data.valves, data.estop, data.fault);
    applyStatusBar(data.fault, data.estop, data.timestamp_str, data.system_status, data.last_seen_str);
    
    $("setValue").textContent = fmt(data.target_o2, 1);
    $("btnEstop").classList.toggle("estop", data.estop);
    $("btnLock").classList.toggle("locked", data.locked_controls);
    $("btnDim").classList.toggle("dim", data.dimmed);
    
    const hist = (data.history || []).filter(d => d.ts >= (Date.now() / 1000 - state.rangeSec));
    drawChart(hist);
    
    if (!$("consoleModal").classList.contains("hidden")) {
        const el = $("consoleText");
        const atBot = el.scrollHeight - el.clientHeight <= el.scrollTop + 10;
        el.textContent = data.console_text || "No console output yet.";
        if (atBot) el.scrollTop = el.scrollHeight;
    }
}

function setupPress(id, d) {
    let t, l = false, el = $(id);
    const dn = e => {
        e.preventDefault();
        l = false;
        t = setTimeout(() => {
            l = true;
            post("adjust_setpoint", { delta: d * 10 }).then(refreshState);
        }, 400);
    };
    const up = e => {
        e.preventDefault();
        clearTimeout(t);
        if (!l) post("adjust_setpoint", { delta: d }).then(refreshState);
    };
    
    el.addEventListener('mousedown', dn);
    el.addEventListener('touchstart', dn, { passive: false });
    el.addEventListener('mouseup', up);
    el.addEventListener('touchend', up);
    el.addEventListener('mouseleave', () => clearTimeout(t));
    el.addEventListener('touchcancel', () => clearTimeout(t));
}

async function init() {
    $("tabAuto").onclick = () => post("set_mode", { mode: "auto" }).then(refreshState);
    $("tabManual").onclick = () => post("set_mode", { mode: "manual" }).then(refreshState);
    
    setupPress("setUp", .1);
    setupPress("setDown", -.1);
    
    $("startStopBtn").onclick = () => post("toggle_auto_running").then(refreshState);
    $("toggleDio0Btn").onclick = () => post("toggle_valve", { valve: "dio0" }).then(refreshState);
    $("toggleDio1Btn").onclick = () => post("toggle_valve", { valve: "dio1" }).then(refreshState);
    $("toggleDio2Btn").onclick = () => post("toggle_valve", { valve: "dio2" }).then(refreshState);
    $("toggleDio3Btn").onclick = () => post("toggle_valve", { valve: "dio3" }).then(refreshState);
    
    document.querySelectorAll(".metricTab").forEach(btn => btn.onclick = () => {
        document.querySelectorAll(".metricTab").forEach(b => b.classList.remove("active"));
        btn.classList.add("active");
        state.metric = btn.dataset.metric;
        if (state.latest) drawChart((state.latest.history || []).filter(d => d.ts >= (Date.now() / 1000 - state.rangeSec)));
    });
    
    document.querySelectorAll(".rangeTab").forEach(btn => btn.onclick = () => {
        document.querySelectorAll(".rangeTab").forEach(b => b.classList.remove("active"));
        btn.classList.add("active");
        state.rangeSec = Number(btn.dataset.range);
        if (state.latest) drawChart((state.latest.history || []).filter(d => d.ts >= (Date.now() / 1000 - state.rangeSec)));
    });
    
    $("csvBtn").onclick = async () => {
        if (!state.latest) await refreshState();
        openQr(state.latest.csv_url);
    };
    
    $("chartCard").onclick = e => {
        if (e.target === $("closeChartBtn")) return;
        $("chartCard").classList.add("fullscreen");
        if (state.latest) drawChart((state.latest.history || []).filter(d => d.ts >= (Date.now() / 1000 - state.rangeSec)));
    };
    
    $("closeChartBtn").onclick = e => {
        e.stopPropagation();
        $("chartCard").classList.remove("fullscreen");
        if (state.latest) drawChart((state.latest.history || []).filter(d => d.ts >= (Date.now() / 1000 - state.rangeSec)));
    };
    
    $("statusBar").onclick = () => openConsole(state.latest && state.latest.console_text || "No console output yet.");
    $("closeConsole").onclick = closeConsole;
    $("closeQr").onclick = closeQr;
    
    $("consoleModal").onclick = e => { if (e.target === $("consoleModal")) closeConsole(); };
    $("qrModal").onclick = e => { if (e.target === $("qrModal")) closeQr(); };
    
    $("btnEstop").onclick = () => { if (confirm("Quit program?")) post("toggle_estop").then(refreshState); };
    $("btnReset").onclick = () => { if (confirm("Reboot Raspberry Pi?")) post("reboot_system").then(refreshState); };
    $("btnDim").onclick = () => post("toggle_dim").then(refreshState);
    $("btnLock").onclick = () => post("toggle_lock").then(refreshState);
    
    await refreshState();
    // Self-rescheduling loop: next poll fires 250ms *after* the previous response
    // arrives, making overlapping in-flight requests structurally impossible.
    (async function pollLoop() {
        while (true) {
            const t0 = performance.now();
            try { await refreshState(); } catch (e) { console.error(e); }
            const wait = Math.max(0, 250 - (performance.now() - t0));
            await new Promise(r => setTimeout(r, wait));
        }
    })();
}

window.addEventListener("DOMContentLoaded", init);
