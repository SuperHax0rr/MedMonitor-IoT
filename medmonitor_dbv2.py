from collections import deque
from threading import Lock, Thread
import time

import serial
from serial import SerialException
from flask import Flask, jsonify, render_template_string

# ---------------- CONFIGURATION ----------------
COM_PORT = "COM5"
BAUD_RATE = 115200

# Firmware sends ECG samples at 125 Hz. Keep the most recent 6 seconds.
ECG_BUFFER_SIZE = 750

app = Flask(__name__)
lock = Lock()
ser = None

ecg_samples = deque(maxlen=ECG_BUFFER_SIZE)
ntc_history = deque(maxlen=180)
bmp_history = deque(maxlen=180)

state = {
    "connected": False,
    "message": "Waiting for ESP32",
    "bmp_status": "Waiting",
    "max_status": "Waiting",
    "ntc_mv": None,
    "ntc_ohms": None,
    "ntc_c": None,
    "bmp_c": None,
    "pressure": None,
    "ir": None,
    "red": None,
    "finger": False,
    "hr": None,
    "spo2": None,
    "ecg_status": "Waiting for ECG samples",
    "last": None,
}

HTML = r"""
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>MEDMONITOR</title>
<style>
:root{--bg:#070a08;--g:#3ddc84;--gd:#1f6b42;--tx:#9fd0b2;--mu:#5f8a70;--amb:#d9b24a;--red:#e8505b}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--tx);font:13px/1.5 ui-monospace,"Cascadia Mono","JetBrains Mono",Consolas,monospace;min-height:100vh;overflow-x:hidden}
body:after{content:"";position:fixed;inset:0;pointer-events:none;background:repeating-linear-gradient(0deg,rgba(0,0,0,.18) 0 1px,transparent 1px 3px)}
.ascii{position:fixed;margin:0;font-size:9px;line-height:1;color:var(--g);opacity:.14;white-space:pre;pointer-events:none}
.ascii.l{left:2vw;top:70px}.ascii.r{right:2vw;top:430px}
main{position:relative;max-width:980px;margin:0 auto;padding:28px 20px 40px}
header{text-align:center}
h1{margin:0;font-size:22px;letter-spacing:.35em;color:#d7f5e3}
.tag{margin:14px auto 24px;display:inline-block;border-left:3px solid var(--g);padding-left:12px;font-style:italic;color:var(--mu)}
.chips{display:flex;gap:12px;justify-content:center;flex-wrap:wrap}
.chip{border:2px solid var(--g);padding:7px 18px;color:#e6fff0;font-size:12px}
.chip em{font-style:normal;margin-left:8px}
.sec{display:flex;align-items:center;gap:12px;margin:36px 0 20px;font-size:11px;letter-spacing:.25em;color:var(--mu)}
.sec i{color:var(--g);font-style:normal;font-size:9px}
.sec hr{flex:1;border:0;border-top:1px solid var(--gd);margin:0}
.tiles{display:flex;flex-wrap:wrap;gap:22px;justify-content:center}
.tile{width:150px;text-align:center}
.box{height:150px;border:1px solid var(--g);padding:10px 6px;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:5px}
.box svg{width:30px;height:30px;stroke:var(--g);fill:none;stroke-width:2;stroke-linecap:round;stroke-linejoin:round}
.val{font-size:30px;font-weight:700;color:#e6fff0;line-height:1.1}
.val small{font-size:11px;color:var(--mu);margin-left:3px;font-weight:400}
.st{font-size:10px;letter-spacing:.08em;color:var(--mu)}
.cap{margin-top:10px;font-size:12px;font-weight:700;letter-spacing:.18em;color:#b8d8c4}
.ok{color:var(--g)}.warn{color:var(--amb)}.bad{color:var(--red)}
.mon{border:2px solid var(--g);border-radius:6px;background:#0b110d;padding:8px}
.bar{display:flex;justify-content:space-between;font-size:9px;letter-spacing:.3em;color:var(--g);padding:0 4px 6px}
.screen{border:2px solid var(--gd);border-radius:4px;background:#020503;padding:12px}
.screen+.screen{margin-top:10px}
.screen h3{margin:0 0 8px;font-size:11px;font-weight:400;color:var(--g)}
canvas{display:block;width:100%;height:170px}
.term{border:1px solid var(--gd);background:#020503;padding:16px 18px;color:var(--g);font-size:12px;white-space:pre-wrap;overflow-wrap:anywhere}
.dim{color:var(--mu)}
.cur{animation:bl 1s steps(2) infinite}
@keyframes bl{50%{opacity:0}}
footer{margin-top:30px;font-size:10px;color:var(--mu);text-align:center;line-height:1.7}
@media(max-width:1250px){.ascii{display:none}}
</style>
</head>
<body>
<pre class="ascii l">
  .:::::.     .:::::.
 :#######:   :#########:
 :#####################:
  :###################:
   ':###############:'
     ':###########:'
       ':#######:'
         ':###:'
           ':'
</pre>
<pre class="ascii r">
                 _
                /|
               / |
 ____    __   /  |    __
     \__/  \_/   |___/  \____
                 \/
</pre>

<main>
<header>
  <h1>MEDMONITOR</h1>
  <div class="tag">vital signs terminal v1.0</div>
  <div class="chips">
    <div class="chip">SERIAL<em id="cs">--</em></div>
    <div class="chip">MAX30102<em id="cm">--</em></div>
    <div class="chip">BMP280<em id="cb">--</em></div>
    <div class="chip">ECG<em id="ce">--</em></div>
  </div>
</header>

<div class="sec"><i>&#9656;</i><span>VITALS</span><hr></div>
<div class="tiles">
  <div class="tile"><div class="box"><svg viewBox="0 0 24 24"><path d="M12 21s-8-5.2-8-11a4.5 4.5 0 0 1 8-2.8A4.5 4.5 0 0 1 20 10c0 5.8-8 11-8 11z"/></svg>
    <div class="val" id="hr">--</div><div class="st" id="hrst">NO SIGNAL</div></div><div class="cap">HEART RATE</div></div>
  <div class="tile"><div class="box"><svg viewBox="0 0 24 24"><path d="M12 3s6 6.5 6 11a6 6 0 0 1-12 0c0-4.5 6-11 6-11z"/></svg>
    <div class="val" id="spo2">--</div><div class="st" id="spo2st">NO SIGNAL</div></div><div class="cap">SPO2</div></div>
  <div class="tile"><div class="box"><svg viewBox="0 0 24 24"><path d="M10 14V5a2 2 0 0 1 4 0v9a4 4 0 1 1-4 0z"/></svg>
    <div class="val" id="ntc">--</div><div class="st warn">UNCALIBRATED</div></div><div class="cap">SKIN TEMP</div></div>
  <div class="tile"><div class="box"><svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="4"/><path d="M12 2v3M12 19v3M2 12h3M19 12h3M5 5l2 2M17 17l2 2M19 5l-2 2M7 17l-2 2"/></svg>
    <div class="val" id="bmpc">--</div><div class="st" id="bmpst">WAITING</div></div><div class="cap">AMBIENT</div></div>
  <div class="tile"><div class="box"><svg viewBox="0 0 24 24"><path d="M4 16a8 8 0 1 1 16 0"/><path d="M12 16l4-5"/></svg>
    <div class="val" id="pressure">--</div><div class="st">STATION</div></div><div class="cap">PRESSURE</div></div>
</div>

<div class="sec"><i>&#9656;</i><span>WAVEFORMS</span><hr></div>
<div class="mon">
  <div class="bar"><span>MEDMONITOR 5100</span><span id="led">&#9679;</span></div>
  <div class="screen"><h3 id="ecgT">ECG &middot; AD8232 &middot; GPIO34 &middot; 125 Hz</h3><canvas id="ecgPlot" width="1200" height="260"></canvas></div>
</div>

<div class="sec"><i>&#9656;</i><span>TEMPERATURE LOG</span><hr></div>
<div class="screen"><h3><span class="ok">&#9632; NTC</span> &nbsp; <span class="warn">&#9632; BMP280</span></h3><canvas id="tempPlot" width="1200" height="260"></canvas></div>

<div class="sec"><i>&#9656;</i><span>SYSTEM LOG</span><hr></div>
<div class="term" id="term">Waiting for ESP32...</div>

<footer>
EDUCATIONAL PROTOTYPE &middot; NOT A MEDICAL DEVICE &middot; DO NOT USE FOR DIAGNOSIS<br>
NTC = local skin-contact temperature, not core body temperature. SpO2 is an uncalibrated estimate.
</footer>
</main>

<script>
const $=id=>document.getElementById(id);
const fmt=(v,n=1)=>v==null?'--':Number(v).toFixed(n);
const esc=s=>String(s).replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]));
const G='#3ddc84',A='#d9b24a';

function setChip(id,text,cls){const e=$(id);e.textContent=text;e.className=cls;}
function setTile(id,v,unit,stId,st,cls){
  $(id).innerHTML=v+(unit?'<small>'+unit+'</small>':'');
  if(stId){const e=$(stId);e.textContent=st;e.className='st '+cls;}
}
function prep(id){
  const cv=$(id),c=cv.getContext('2d'),w=cv.width,h=cv.height;
  c.clearRect(0,0,w,h);c.strokeStyle='#0f2a1b';c.lineWidth=1;
  for(let i=1;i<10;i++){const x=i*w/10;c.beginPath();c.moveTo(x,0);c.lineTo(x,h);c.stroke();}
  for(let i=1;i<5;i++){const y=i*h/5;c.beginPath();c.moveTo(0,y);c.lineTo(w,y);c.stroke();}
  return [c,w,h];
}
function msg(c,t){c.fillStyle='#5f8a70';c.font='15px monospace';c.fillText(t,16,30);}
function line(c,w,h,vals,lo,hi,color){
  if(!vals||vals.length<2)return;
  c.strokeStyle=color;c.lineWidth=2;c.shadowColor=color;c.shadowBlur=8;c.beginPath();
  let on=false;
  vals.forEach((v,i)=>{
    if(v==null||!Number.isFinite(v))return;
    const x=i*w/(vals.length-1),y=h-(v-lo)/(hi-lo)*h;
    on?c.lineTo(x,y):c.moveTo(x,y);on=true;
  });
  c.stroke();c.shadowBlur=0;
}
function drawECG(s){
  const [c,w,h]=prep('ecgPlot');
  if(!s||s.length<2){msg(c,'WAITING FOR ECG SAMPLES...');return;}
  line(c,w,h,s,0,4095,G);
}
function drawTemps(a,b){
  const [c,w,h]=prep('tempPlot');
  const all=a.concat(b).filter(v=>v!=null&&Number.isFinite(v));
  if(!all.length){msg(c,'WAITING FOR TEMPERATURE DATA...');return;}
  let lo=Math.min(...all)-2,hi=Math.max(...all)+2;if(hi-lo<1){lo-=1;hi+=1;}
  line(c,w,h,a,lo,hi,G);line(c,w,h,b,lo,hi,A);
}
const row=(k,v)=>'&gt; '+esc(k)+'<span class="dim">'+'.'.repeat(Math.max(2,16-k.length))+'</span>['+esc(v)+']\n';

async function update(){
  try{
    const d=await (await fetch('/api/data',{cache:'no-store'})).json();
    const sat=d.ir!=null&&d.ir>=250000;

    setChip('cs',d.connected?'ONLINE':'OFFLINE',d.connected?'ok':'bad');
    const dev=s=>s==='Detected'?['OK','ok']:s==='Not detected'?['FAULT','bad']:['WAIT','warn'];
    setChip('cm',...dev(d.max_status));setChip('cb',...dev(d.bmp_status));
    const ecgLive=d.ecg_status.startsWith('Receiving'),lead=d.ecg_status.includes('lead-off');
    setChip('ce',ecgLive?'LIVE':lead?'LEAD OFF':'WAIT',ecgLive?'ok':lead?'bad':'warn');
    $('led').style.color=d.connected?G:'#e8505b';

    // vitals
    const hrSt=d.hr!=null?['SIGNAL OK','ok']:sat?['IR SATURATED','bad']:d.finger?['ACQUIRING...','warn']:['NO FINGER',''];
    setTile('hr',d.hr==null?'--':d.hr,d.hr==null?'':'BPM','hrst',...hrSt);
    const spSt=d.spo2!=null?['ESTIMATE','warn']:d.finger?['ACQUIRING...','warn']:['NO FINGER',''];
    setTile('spo2',d.spo2==null?'--':d.spo2,d.spo2==null?'':'%','spo2st',...spSt);
    setTile('ntc',fmt(d.ntc_c,1),'&deg;C');
    setTile('bmpc',fmt(d.bmp_c,1),'&deg;C','bmpst',d.bmp_c==null?'NO DATA':'LIVE',d.bmp_c==null?'bad':'ok');
    setTile('pressure',fmt(d.pressure,0),'hPa');

    // ring
    drawECG(d.ecg);drawTemps(d.ntc_history,d.bmp_history);

    $('term').innerHTML='MEDMONITOR v1.0 - SYSTEM LOG\n'+'-'.repeat(34)+'\n'+
      row('SERIAL',d.connected?'CONNECTED':'DISCONNECTED')+
      row('MAX30102',d.max_status.toUpperCase())+
      row('BMP280',d.bmp_status.toUpperCase())+
      row('ECG',d.ecg_status.toUpperCase())+
      row('IR / RED',(d.ir==null?'--':d.ir.toLocaleString())+' / '+(d.red==null?'--':d.red.toLocaleString()))+
      row('FINGER',d.finger?'DETECTED':'NOT DETECTED')+
      row('NTC RAW',fmt(d.ntc_mv,0)+' mV, '+fmt(d.ntc_ohms,0)+' ohm')+
      row('LAST PACKET',d.last_text)+
      '<span class="dim">&gt; '+esc(d.message)+'</span>\n<span class="cur">_</span>';
  }catch(e){
    $('term').textContent='> DASHBOARD API ERROR: '+e;
  }
}
setInterval(update,250);update();
</script>
</body></html>
"""

def serial_reader():
    global ser

    while True:
        try:
            if ser is None or not ser.is_open:
                with lock:
                    state["connected"] = False
                    state["message"] = f"Opening {COM_PORT}..."

                ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
                # Allow the ESP32 to reset after the serial port opens.
                time.sleep(2)

                with lock:
                    state["connected"] = True
                    state["message"] = f"Connected to {COM_PORT} at {BAUD_RATE} baud"

            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            if line.startswith("#BMP280:"):
                with lock:
                    state["bmp_status"] = "Detected" if line.endswith("OK") else "Not detected"
                continue

            if line.startswith("#MAX30102:"):
                with lock:
                    state["max_status"] = "Detected" if line.endswith("OK") else "Not detected"
                continue

            # ECG stream is separate from the once-per-second DATA packet.
            if line.startswith("ECG,"):
                value = line[4:].strip()
                with lock:
                    if value == "LEADOFF":
                        state["ecg_status"] = "Electrode lead-off detected"
                    else:
                        try:
                            ecg_samples.append(int(value))
                            state["ecg_status"] = "Receiving ECG samples"
                        except ValueError:
                            state["message"] = f"Ignored invalid ECG sample: {value}"
                continue

            if line.startswith("DATA,"):
                fields = line.split(",")
                # DATA plus 12 values = 13 total fields.
                if len(fields) != 13:
                    with lock:
                        state["message"] = f"Skipped malformed DATA packet: got {len(fields)} fields, expected 13"
                    continue

                try:
                    values = [float(value) for value in fields[1:]]
                except ValueError:
                    with lock:
                        state["message"] = "Skipped DATA packet containing a non-numeric value"
                    continue

                (mv, ohms, ntc, bmp_c, pressure, ir, red, finger,
                 hr, hr_valid, spo2, spo2_valid) = values

                def valid_or_none(value):
                    return value if value > -900 else None

                with lock:
                    state.update({
                        "ntc_mv": valid_or_none(mv),
                        "ntc_ohms": valid_or_none(ohms),
                        "ntc_c": valid_or_none(ntc),
                        "bmp_c": valid_or_none(bmp_c),
                        "pressure": valid_or_none(pressure),
                        "ir": int(ir),
                        "red": int(red),
                        "finger": bool(int(finger)),
                        "hr": int(hr) if int(hr_valid) == 1 and 30 <= hr <= 220 else None,
                        "spo2": int(spo2) if int(spo2_valid) == 1 and 70 <= spo2 <= 100 else None,
                        "last": time.time(),
                    })
                    ntc_history.append(state["ntc_c"])
                    bmp_history.append(state["bmp_c"])

        except (SerialException, OSError) as exc:
            with lock:
                state["connected"] = False
                state["message"] = f"Serial error: {exc}"

            try:
                if ser is not None:
                    ser.close()
            except Exception:
                pass

            ser = None
            time.sleep(2)


@app.route("/")
def home():
    return render_template_string(HTML)


@app.route("/api/data")
def api_data():
    with lock:
        result = dict(state)
        result["ecg"] = list(ecg_samples)
        result["ntc_history"] = list(ntc_history)
        result["bmp_history"] = list(bmp_history)
        result["last_text"] = (
            "No DATA packets received"
            if state["last"] is None
            else f"{time.time() - state['last']:.1f}s ago"
        )
        return jsonify(result)


if __name__ == "__main__":
    Thread(target=serial_reader, daemon=True).start()
    app.run(host="127.0.0.1", port=5000, debug=False, threaded=True)
