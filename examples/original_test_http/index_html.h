#pragma once

#include <Arduino.h>

static const char kIndexHtml[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>T-2Can Control Center</title>
<style>
body{margin:0;font-family:Arial,sans-serif;background:#101418;color:#e8eef2}
header{padding:22px 24px;background:#17212b;border-bottom:1px solid #2a3a48}
h1{margin:0;font-size:24px}main{padding:18px;display:grid;gap:14px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(240px,1fr));gap:14px}
.card{background:#18222c;border:1px solid #2b3a47;border-radius:8px;padding:16px}
.label{color:#91a4b4;font-size:13px}.value{font-size:26px;margin-top:6px}
.row{display:flex;gap:8px;flex-wrap:wrap;margin-top:12px;align-items:center}
button{background:#263746;color:#e8eef2;border:1px solid #41566a;border-radius:6px;padding:10px 14px;cursor:pointer}
button.active{background:#1f8f5f;border-color:#32b878}
.ok{color:#63d68a}.bad{color:#ff7c7c}.small{font-size:14px;color:#a9bac7;margin-top:6px}
pre{white-space:pre-wrap;word-break:break-word;border:1px solid #2d3942;border-radius:6px;padding:12px;max-height:260px;overflow:auto;background:#05080a;font-family:Consolas,monospace;font-size:13px;margin:10px 0 0}
a{color:#8cc7ff;text-decoration:none}
</style>
</head>
<body>
<header>
  <h1 id="title">T-2Can Control Center</h1>
  <div class="small" id="subtitle">Wi-Fi AP/STA and Dual CAN (CAN V2.0B 500kbps) are enabled</div>
</header>
<main>
  <section class="grid">
    <div class="card"><div class="label">AP</div><div class="value" id="ap">-</div><div class="small" id="apip"></div></div>
    <div class="card"><div class="label">STA</div><div class="value" id="sta">-</div><div class="small" id="staip"></div></div>
    <div class="card"><div class="label">Time</div><div class="value" id="time">-</div><div class="small" id="timeage"></div></div>
  </section>
  <section class="grid">
    <div class="card">
      <div class="label" id="canalabel">CAN A</div>
      <div class="value" id="cana">-</div>
      <div class="small" id="canainfo"></div>
      <div class="small" id="canarx"></div>
      <div class="row">
        <button id="cana-auto" onclick="setMode('can_a','auto')">Auto</button>
        <button id="cana-send" onclick="setMode('can_a','send')">Send</button>
        <button id="cana-receive" onclick="setMode('can_a','receive')">Receive</button>
        <button id="cana-stop" onclick="setMode('can_a','stop')">Stop</button>
      </div>
    </div>
    <div class="card">
      <div class="label" id="canblabel">CAN B</div>
      <div class="value" id="canb">-</div>
      <div class="small" id="canbinfo"></div>
      <div class="small" id="canbrx"></div>
      <div class="row">
        <button id="canb-auto" onclick="setMode('can_b','auto')">Auto</button>
        <button id="canb-send" onclick="setMode('can_b','send')">Send</button>
        <button id="canb-receive" onclick="setMode('can_b','receive')">Receive</button>
        <button id="canb-stop" onclick="setMode('can_b','stop')">Stop</button>
      </div>
    </div>
  </section>
  <section class="card">
    <div class="row" style="margin-top:0;justify-content:space-between">
      <div class="label">Serial Log</div>
      <div class="row" style="margin-top:0">
        <a href="/log" target="_blank">raw text</a>
        <button onclick="clearLog()">Clear</button>
      </div>
    </div>
    <pre id="log"></pre>
  </section>
</main>
<script>
function setBtns(prefix,mode){
  ['auto','send','receive','stop'].forEach(m=>{
    const el=document.getElementById(`${prefix}-${m}`);
    if(el) el.className=(mode===m)?'active':'';
  });
}
async function setMode(target,mode){
  await fetch(`/api/set?target=${target}&mode=${mode}`);
  refresh();
}
async function clearLog(){
  await fetch('/clear');
  refresh();
}
async function refresh(){
  const [s,logText]=await Promise.all([
    fetch('/api/status').then(r=>r.json()),
    fetch('/log').then(r=>r.text())
  ]);
  document.getElementById('title').textContent=`${s.board} Control Center`;
  document.getElementById('subtitle').textContent=`${s.software} ${s.version} | ${s.can_a.protocol} ${s.can_a.speed}`;
  document.getElementById('ap').textContent=s.wifi.ap_ssid;
  document.getElementById('apip').textContent=s.wifi.ap_ip;
  const sta=document.getElementById('sta');
  sta.textContent=s.wifi.sta?s.wifi.sta_ssid:'Connecting';
  sta.className='value '+(s.wifi.sta?'ok':'bad');
  document.getElementById('staip').textContent=s.wifi.sta?`${s.wifi.sta_ip} | RSSI ${s.wifi.sta_rssi} dBm`:s.wifi.sta_ip;
  document.getElementById('time').textContent=s.time;
  document.getElementById('timeage').textContent=`updated ${s.time_age_seconds} s ago`;

  document.getElementById('canalabel').textContent=`CAN A (${s.can_a.module}) | ${s.can_a.protocol} ${s.can_a.speed}`;
  const cana=document.getElementById('cana');
  cana.textContent=s.can_a.init?(s.can_a.mode.charAt(0).toUpperCase()+s.can_a.mode.slice(1)):'Init Failed';
  cana.className='value '+(s.can_a.init&&s.can_a.errors===0?'ok':'bad');
  document.getElementById('canainfo').textContent=`TX ${s.can_a.tx} | RX ${s.can_a.rx} (${s.can_a.rx_bytes} B) | errors ${s.can_a.errors}`;
  document.getElementById('canarx').textContent=`last RX id ${s.can_a.last_id} | data [${s.can_a.last_data}]`;
  setBtns('cana',s.can_a.mode);

  document.getElementById('canblabel').textContent=`CAN B (${s.can_b.module}) | ${s.can_b.protocol} ${s.can_b.speed}`;
  const canb=document.getElementById('canb');
  canb.textContent=s.can_b.init?(s.can_b.mode.charAt(0).toUpperCase()+s.can_b.mode.slice(1)):'Init Failed';
  canb.className='value '+(s.can_b.init&&s.can_b.state==='active'?'ok':'bad');
  document.getElementById('canbinfo').textContent=`state ${s.can_b.state} | TX ${s.can_b.tx} | RX ${s.can_b.rx} (${s.can_b.rx_bytes} B) | bus error ${s.can_b.errors}`;
  document.getElementById('canbrx').textContent=`last RX id ${s.can_b.last_id} | data [${s.can_b.last_data}]`;
  setBtns('canb',s.can_b.mode);

  const logEl=document.getElementById('log');
  logEl.textContent=logText;
  logEl.scrollTop=logEl.scrollHeight;
}
setInterval(refresh,1000);refresh();
</script>
</body>
</html>
)HTML";
