#include "Web.h"
#include "Globals.h"
#include <DNSServer.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>

// Removed changeState extern

const byte DNS_PORT = 53;
DNSServer dnsServer;
WebServer server(80);
Preferences settings;
const char *ssid = "ROBOT_VANG_100";
const char *password = "chaydiemmuoi";

IrSnapshot latestRawIrSnapshot() {
  IrSnapshot s;
  s.left = raw_ir_L;
  s.frontLeft = raw_ir_FL;
  s.frontRight = raw_ir_FR;
  s.right = raw_ir_R;
  return s;
}

void clampRuntimeParams() {
  Turn_Min = constrain(Turn_Min, 0, 255);
  Turn_Max = constrain(Turn_Max, Turn_Min, 255);
  base_pwm = constrain(base_pwm, Turn_Min, Turn_Max);
  ramp_rate = constrain(ramp_rate, 1, 255);
  min_vel = max(0.0f, min_vel);
  max_vel = max(min_vel, max_vel);
  accel_rate = max(0.01f, accel_rate);
  k_ir = max(0.0f, k_ir);
  ir_deadband = max(0, ir_deadband);
  offset_upper = max(0, offset_upper);
  offset_lower = max(0, offset_lower);
  pulses_per_cell = max(1, pulses_per_cell);
  side_ref_L = constrain(side_ref_L, 0, 4095);
  side_ref_FL = constrain(side_ref_FL, 0, 4095);
  side_ref_FR = constrain(side_ref_FR, 0, 4095);
  side_ref_R = constrain(side_ref_R, 0, 4095);
}

void loadRuntimeParams() {
  settings.begin("robot_pid", true);
  offset_upper = settings.getInt("offUp", offset_upper);
  offset_lower = settings.getInt("offLow", offset_lower);
  ir_deadband = settings.getInt("deadband", ir_deadband);
  base_pwm = settings.getInt("base", base_pwm);
  Kp_L = settings.getFloat("kpL", Kp_L);
  Ki_L = settings.getFloat("kiL", Ki_L);
  Kd_L = settings.getFloat("kdL", Kd_L);
  Kp_R = settings.getFloat("kpR", Kp_R);
  Ki_R = settings.getFloat("kiR", Ki_R);
  Kd_R = settings.getFloat("kdR", Kd_R);
  accel_rate = settings.getFloat("accel", accel_rate);
  max_vel = settings.getFloat("vmax", max_vel);
  min_vel = settings.getFloat("vmin", min_vel);
  ramp_rate = settings.getInt("ramp", ramp_rate);
  k_gyro = settings.getFloat("gyro", k_gyro);
  k_ir = settings.getFloat("irGain", k_ir);
  Turn_Min = settings.getInt("pwmMin", Turn_Min);
  Turn_Max = settings.getInt("pwmMax", Turn_Max);
  pulses_per_cell = settings.getInt("xungO", pulses_per_cell);
  side_ref_L = settings.getInt("side7L", side_ref_L);
  side_ref_FL = settings.getInt("side7FL", side_ref_FL);
  side_ref_FR = settings.getInt("side7FR", side_ref_FR);
  side_ref_R = settings.getInt("side7R", side_ref_R);
  if (side_ref_FL >= 4095 && side_ref_L < 4095)
    side_ref_FL = side_ref_L;
  if (side_ref_FR >= 4095 && side_ref_R < 4095)
    side_ref_FR = side_ref_R;
  for (int s = 1; s <= 4; s++) {
    char key[8];
    // store/load using 1-based sensor numbering (max1..max4) but map to internal 0-based arrays
    snprintf(key, sizeof(key), "max%d", s);
    max_IR[SIDX(s)] = settings.getInt(key, max_IR[SIDX(s)]);
    snprintf(key, sizeof(key), "min%d", s);
    min_IR[SIDX(s)] = settings.getInt(key, min_IR[SIDX(s)]);
  }
  settings.end();
  clampRuntimeParams();
}

void saveRuntimeParams() {
  clampRuntimeParams();
  settings.begin("robot_pid", false);
  settings.putInt("offUp", offset_upper);
  settings.putInt("offLow", offset_lower);
  settings.putInt("deadband", ir_deadband);
  settings.putInt("base", base_pwm);
  settings.putFloat("kpL", Kp_L);
  settings.putFloat("kiL", Ki_L);
  settings.putFloat("kdL", Kd_L);
  settings.putFloat("kpR", Kp_R);
  settings.putFloat("kiR", Ki_R);
  settings.putFloat("kdR", Kd_R);
  settings.putFloat("accel", accel_rate);
  settings.putFloat("vmax", max_vel);
  settings.putFloat("vmin", min_vel);
  settings.putInt("ramp", ramp_rate);
  settings.putFloat("gyro", k_gyro);
  settings.putFloat("irGain", k_ir);
  settings.putInt("pwmMin", Turn_Min);
  settings.putInt("pwmMax", Turn_Max);
  settings.putInt("xungO", pulses_per_cell);
  settings.putInt("side7L", side_ref_L);
  settings.putInt("side7FL", side_ref_FL);
  settings.putInt("side7FR", side_ref_FR);
  settings.putInt("side7R", side_ref_R);
  for (int s = 1; s <= 4; s++) {
    char key[8];
    // use 1-based sensor numbering for persistent keys
    snprintf(key, sizeof(key), "max%d", s);
    settings.putInt(key, max_IR[SIDX(s)]);
    snprintf(key, sizeof(key), "min%d", s);
    settings.putInt(key, min_IR[SIDX(s)]);
  }
  settings.end();
}

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width, initial-scale=1" charset="UTF-8">
<title>DEBUG ADC - MOTOR TEST</title>
<style>
  body { font-family: 'Segoe UI', Tahoma, sans-serif; background-color: #f0f2f5; color: #333; padding: 5px; margin: 0; min-height: 100vh; overflow-y: auto; }
  h2 { text-align: center; color: #2c3e50; margin: 5px 0; font-size: 16px; font-weight: bold;}
  .half-top { background: #fff; padding: 10px; border-radius: 8px; margin-bottom: 5px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }
  .half-bot { height: 40vh; min-height: 260px; background: #fff; padding: 5px; border-radius: 8px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); display: flex; flex-direction: column; }
  .grid-2 { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; margin-bottom: 8px;}
  .group { background: #e8f4f8; padding: 6px; border-radius: 5px; border-left: 4px solid #3498db;}
  .group h4 { margin: 0 0 5px 0; color: #2980b9; text-align: center; font-size: 12px;}
  label { display: flex; justify-content: space-between; align-items: center; margin-bottom: 4px; font-size: 11px; font-weight: bold;}
  input[type=number] { width: 50px; padding: 3px; border: 1px solid #ccc; text-align: center; font-weight: bold; border-radius: 3px;}
  
  .btn { width: 100%; padding: 8px; font-size: 12px; font-weight: bold; color: #fff; border: none; border-radius: 5px; cursor: pointer; margin-bottom: 5px;}
  .btn-calib-h { background: #9b59b6; }
  .btn-calib-l { background: #f39c12; }
  .btn-save { background: #34495e; }
  .grid-btn { display: grid; grid-template-columns: 1fr 1fr; gap: 5px; }
  .btn-run { background: #2ecc71; }
  .btn-stop { background: #e74c3c; }
  
  .val-box { display: grid; grid-template-columns: 1fr 1fr; gap: 4px; font-size: 11px; text-align: left; }
  .val-item { background: #fff; padding: 3px; border-radius: 3px; border: 1px solid #ccc; }
  .raw { color: #2980b9; font-weight: bold; }
  .delta { color: #c0392b; font-weight: bold; }

  #console { height: 35px; overflow-y: auto; font-size: 11px; color: #333; background: #ecf0f1; border: 1px solid #bdc3c7; padding: 3px; border-radius: 3px;}
  #chartCanvas { width: 100%; flex: 1; background: #fff; border: 1px solid #bdc3c7; margin-top: 3px; border-radius: 3px;}
</style>
</head><body>
  <div class="half-top">
    <h2>[ HỆ THỐNG GỠ RỐI PHẦN CỨNG ]</h2>
    <div class="grid-2">
      <div class="group">
        <h4>OFFSET & DEADBAND</h4>
        <label>- Ngưỡng Tường <input type="number" id="off_up" value="500"></label>
        <label>+ Ngưỡng Trống <input type="number" id="off_low" value="700"></label>
        <label>Deadband <input type="number" id="deadband" value="150"></label>
        <label>PWM Gốc <input type="number" id="basepwm" value="80"></label>
      </div>
      <div class="group">
        <h4>ADC MẮT THẦN (RAW / SAU OFFSET)</h4>
        <div class="val-box">
          <div class="val-item">L: <span id="r_l" class="raw">0</span> / <span id="d_l" class="delta">0</span></div>
          <div class="val-item">FL: <span id="r_fl" class="raw">0</span> / <span id="d_fl" class="delta">0</span></div>
          <div class="val-item">FR: <span id="r_fr" class="raw">0</span> / <span id="d_fr" class="delta">0</span></div>
          <div class="val-item">R: <span id="r_r" class="raw">0</span> / <span id="d_r" class="delta">0</span></div>
        </div>
      </div>
    </div>

    <div class="grid-2">
      <div class="group">
        <h4>PID TOC DO TRAI</h4>
        <label>P <input type="number" id="kp_l" step="0.001" value="5.025"></label>
        <label>I <input type="number" id="ki_l" step="0.001" value="0.0"></label>
        <label>D <input type="number" id="kd_l" step="0.001" value="15.1"></label>
      </div>
      <div class="group">
        <h4>PID TOC DO PHAI</h4>
        <label>P <input type="number" id="kp_r" step="0.001" value="5.0"></label>
        <label>I <input type="number" id="ki_r" step="0.001" value="0.0"></label>
        <label>D <input type="number" id="kd_r" step="0.001" value="15.1"></label>
      </div>
    </div>

    <div class="grid-2">
      <div class="group">
        <h4>PROFILE TOC DO</h4>
        <label>G.Toc <input type="number" id="accel" step="0.01" value="0.2"></label>
        <label>V.Max <input type="number" id="vmax" step="0.1" value="15.0"></label>
        <label>V.Min <input type="number" id="vmin" step="0.1" value="5.0"></label>
      </div>
      <div class="group">
        <h4>KHUNG GAM</h4>
        <label>Ramp <input type="number" id="ramp" step="1" value="10"></label>
        <label>Gyro <input type="number" id="gyro" step="0.1" value="6.0"></label>
        <label>IR Gain <input type="number" id="ir_gain" step="0.01" value="0.1"></label>
        <label>PWM Min <input type="number" id="pwm_min" step="1" value="35"></label>
        <label>PWM Max <input type="number" id="pwm_max" step="1" value="100"></label>
        <label>Xung/20cm <input type="number" id="xung_o" step="1" value="980"></label>
      </div>
    </div>
    
    <div class="grid-2">
      <button class="btn btn-calib-h" onclick="calib('High').then(r=>r.text())">🎯 SET TƯỜNG (RAW - OFF)</button>
      <button class="btn btn-calib-l" onclick="calib('Low').then(r=>r.text())">🌌 SET TRỐNG (RAW + OFF)</button>
    </div>
    <div class="grid-2">
      <button class="btn" style="background:#16a085;" onclick="calib('SideL7').then(r=>r.text())">SET FL 7CM</button>
      <button class="btn" style="background:#16a085;" onclick="calib('SideR7').then(r=>r.text())">SET FR 7CM</button>
    </div>
    <button class="btn" style="background:#117a65;" onclick="calib('Side7').then(r=>r.text())">SET L/R 7CM</button>
    <button class="btn btn-save" onclick="saveAll()">💾 LƯU THÔNG SỐ BĂM XUNG</button>
    
    <div class="grid-2">
      <button class="btn" style="background:#8e44ad;" onclick="fetch('/cmd?val=TEST_L').then(r=>r.text())">TEST MOTOR TRAI</button>
      <button class="btn" style="background:#d35400;" onclick="fetch('/cmd?val=TEST_R').then(r=>r.text())">TEST MOTOR PHAI</button>
    </div>

    <div class="grid-btn">
      <button class="btn btn-run" onclick="fetch('/cmd?val=START')">CHAY LIEN TUC</button>
      <button class="btn btn-run" style="background:#27ae60;" onclick="fetch('/cmd?val=ONE_CELL')">DI THANG 1 O</button>
      <button class="btn btn-stop" onclick="fetch('/cmd?val=STOP')">🛑 PHANH</button>
    </div>
  </div>
  
  <div class="half-bot">
    <div style="font-weight: bold; font-size: 11px; margin-bottom: 2px;">LIVE GRAPH (ADC / ENCODER)</div>
    <div id="console"></div>
    <canvas id="chartCanvas"></canvas>
  </div>

<script>
  const maxPts = 40; let dataBuf = { irL:[], irFL:[], irFR:[], irR:[], encL:[], encR:[] };
  let canvas = document.getElementById('chartCanvas'); let ctx = canvas.getContext('2d');
  let paramsSynced = false;
  let lastViewEncL = null, lastViewEncR = null;
  const paramFields = {
    offUp: 'off_up', offLow: 'off_low', deadband: 'deadband', base: 'basepwm',
    kpL: 'kp_l', kiL: 'ki_l', kdL: 'kd_l', kpR: 'kp_r', kiR: 'ki_r',
    kdR: 'kd_r', accel: 'accel', vmax: 'vmax', vmin: 'vmin',
    ramp: 'ramp', gyro: 'gyro', irGain: 'ir_gain', pwmMin: 'pwm_min', pwmMax: 'pwm_max',
    xungO: 'xung_o'
  };
  function resizeCanvas() { canvas.width = canvas.clientWidth; canvas.height = canvas.clientHeight; }
  window.addEventListener('resize', resizeCanvas); setTimeout(resizeCanvas, 100);

  function syncParamInputs(d) {
    if(paramsSynced) return;
    for(const key in paramFields) {
      if(d[key] !== undefined) document.getElementById(paramFields[key]).value = d[key];
    }
    paramsSynced = true;
  }

  function drawChart() {
    if(canvas.width === 0) resizeCanvas();
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    if(dataBuf.irL.length === 0) return;
    let w = canvas.width; let h = canvas.height;
    
    let maxEnc = -999999, minEnc = 999999;
    for(let i=0; i<dataBuf.encL.length; i++) {
        if(dataBuf.encL[i] > maxEnc) maxEnc = dataBuf.encL[i]; if(dataBuf.encL[i] < minEnc) minEnc = dataBuf.encL[i];
        if(dataBuf.encR[i] > maxEnc) maxEnc = dataBuf.encR[i]; if(dataBuf.encR[i] < minEnc) minEnc = dataBuf.encR[i];
    }
    if(maxEnc === minEnc) { maxEnc += 10; minEnc -= 10; }

    ctx.strokeStyle = '#eee'; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(0, h/2); ctx.lineTo(w, h/2); ctx.stroke();

    let drawLine = (arr, color, isEnc) => {
        ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = 2;
        let stepX = w / (maxPts - 1);
        for(let i=0; i<arr.length; i++) {
            let x = i * stepX; let y;
            if(isEnc) y = h - ((arr[i] - minEnc) / (maxEnc - minEnc)) * h;
            else y = h - (arr[i] / 4095) * h;
            if(i===0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        }
        ctx.stroke();
    };

    drawLine(dataBuf.irL, '#3498db', false); drawLine(dataBuf.irFL, '#2ecc71', false);
    drawLine(dataBuf.irFR, '#f39c12', false); drawLine(dataBuf.irR, '#9b59b6', false);
    drawLine(dataBuf.encL, '#e74c3c', true); drawLine(dataBuf.encR, '#2c3e50', true);
  }

  function saveAll() {
    const val = id => encodeURIComponent(document.getElementById(id).value);
    let q = [
      "up=" + val('off_up'),
      "low=" + val('off_low'),
      "db=" + val('deadband'),
      "base=" + val('basepwm'),
      "kpL=" + val('kp_l'),
      "kiL=" + val('ki_l'),
      "kdL=" + val('kd_l'),
      "kpR=" + val('kp_r'),
      "kiR=" + val('ki_r'),
      "kdR=" + val('kd_r'),
      "accel=" + val('accel'),
      "vmax=" + val('vmax'),
      "vmin=" + val('vmin'),
      "ramp=" + val('ramp'),
      "gyro=" + val('gyro'),
      "irGain=" + val('ir_gain'),
      "pwmMin=" + val('pwm_min'),
      "pwmMax=" + val('pwm_max'),
      "xungO=" + val('xung_o')
    ].join("&");
    fetch('/set?' + q)
      .then(r => r.text())
      .then(() => {
        paramsSynced = false;
        document.getElementById('console').innerHTML = 'Saved PID and run params';
      });
  }

  function calib(type) {
    return fetch('/calib' + type)
      .then(response => {
        response.clone().text().then(text => { alert("✅ " + text); });
        return response;
      })
      .catch(err => alert("❌ Lỗi rớt mạng WiFi!"));
  }

  let isPaused = false;
  
  function fetchLoop() {
    if(!isPaused) {
      fetch('/data').then(r => r.json()).then(d => {
          syncParamInputs(d);
          for(let key in d) {
              if(dataBuf[key] !== undefined) {
                  dataBuf[key].push(d[key]);
                  if(dataBuf[key].length > maxPts) dataBuf[key].shift();
              }
          }
          drawChart();
          
          // Cập nhật 8 ô ADC
          document.getElementById('r_l').innerText = d.irL; document.getElementById('d_l').innerText = d.dL;
          document.getElementById('r_fl').innerText = d.irFL; document.getElementById('d_fl').innerText = d.dFL;
          document.getElementById('r_fr').innerText = d.irFR; document.getElementById('d_fr').innerText = d.dFR;
          document.getElementById('r_r').innerText = d.irR; document.getElementById('d_r').innerText = d.dR;

          let spdL = lastViewEncL === null ? 0 : Math.abs(d.encL - lastViewEncL);
          let spdR = lastViewEncR === null ? 0 : Math.abs(d.encR - lastViewEncR);
          lastViewEncL = d.encL; lastViewEncR = d.encR;

          document.getElementById('console').innerHTML = `<span style="color:#c0392b; font-weight:bold;">ENC_L:</span> ${d.encL} | <span style="color:#2c3e50; font-weight:bold;">ENC_R:</span> ${d.encR} | SPD: ${spdL}/${spdR} | PWM: ${d.pwmL}/${d.pwmR} | SIDE: FL ${d.irFL}/${d.sideFL} e${d.errFL} - FR ${d.irFR}/${d.sideFR} e${d.errFR} | STEER: ${d.steerIR}/${d.totalSteer} | STATE: ${d.state}`;
          
          setTimeout(fetchLoop, 100); 
        }).catch(e => { setTimeout(fetchLoop, 500); });
    } else { setTimeout(fetchLoop, 100); }
  }
  fetchLoop();
</script>
</body></html>
)rawliteral";

void handleRoot() { server.send(200, "text/html", index_html); }

void handleSet() {
  if (server.hasArg("up")) offset_upper = server.arg("up").toInt();
  if (server.hasArg("low")) offset_lower = server.arg("low").toInt();
  if (server.hasArg("db")) ir_deadband = server.arg("db").toInt();
  if (server.hasArg("base")) base_pwm = server.arg("base").toInt();
  if (server.hasArg("kpL")) Kp_L = server.arg("kpL").toFloat();
  if (server.hasArg("kiL")) Ki_L = server.arg("kiL").toFloat();
  if (server.hasArg("kdL")) Kd_L = server.arg("kdL").toFloat();
  if (server.hasArg("kpR")) Kp_R = server.arg("kpR").toFloat();
  if (server.hasArg("kiR")) Ki_R = server.arg("kiR").toFloat();
  if (server.hasArg("kdR")) Kd_R = server.arg("kdR").toFloat();
  if (server.hasArg("accel")) accel_rate = server.arg("accel").toFloat();
  if (server.hasArg("vmax")) max_vel = server.arg("vmax").toFloat();
  if (server.hasArg("vmin")) min_vel = server.arg("vmin").toFloat();
  if (server.hasArg("ramp")) ramp_rate = server.arg("ramp").toInt();
  if (server.hasArg("gyro")) k_gyro = server.arg("gyro").toFloat();
  if (server.hasArg("irGain")) k_ir = server.arg("irGain").toFloat();
  if (server.hasArg("pwmMin")) Turn_Min = server.arg("pwmMin").toInt();
  if (server.hasArg("pwmMax")) Turn_Max = server.arg("pwmMax").toInt();
  if (server.hasArg("xungO")) pulses_per_cell = server.arg("xungO").toInt();
  saveRuntimeParams();
  server.send(200, "text/plain", "OK");
}

void handleCalibHigh() {
  max_IR[SIDX(S_L)] = constrain(ir_L - offset_upper, 0, 4095);
  max_IR[SIDX(S_FL)] = constrain(ir_FL - offset_upper, 0, 4095);
  max_IR[SIDX(S_FR)] = constrain(ir_FR - offset_upper, 0, 4095);
  max_IR[SIDX(S_R)] = constrain(ir_R - offset_upper, 0, 4095);
  saveRuntimeParams();
  server.send(200, "text/plain", "Đã chốt Ngưỡng Tường (Max)!");
}

void handleCalibLow() {
  min_IR[SIDX(S_L)] = constrain(ir_L + offset_lower, 0, 4095);
  min_IR[SIDX(S_FL)] = constrain(ir_FL + offset_lower, 0, 4095);
  min_IR[SIDX(S_FR)] = constrain(ir_FR + offset_lower, 0, 4095);
  min_IR[SIDX(S_R)] = constrain(ir_R + offset_lower, 0, 4095);
  saveRuntimeParams();
  server.send(200, "text/plain", "Đã chốt Ngưỡng Trống (Min)!");
}

void handleCalibSide7() {
  IrSnapshot liveIr = latestRawIrSnapshot();
  side_ref_L = constrain(liveIr.left, 0, 4095);
  side_ref_FL = constrain(liveIr.frontLeft, 0, 4095);
  side_ref_FR = constrain(liveIr.frontRight, 0, 4095);
  side_ref_R = constrain(liveIr.right, 0, 4095);
  saveRuntimeParams();
  server.send(200, "text/plain", "Da chot moc L/FL/FR/R 7cm!");
}

void handleCalibSideL7() {
  IrSnapshot liveIr = latestRawIrSnapshot();
  side_ref_FL = constrain(liveIr.frontLeft, 0, 4095);
  saveRuntimeParams();
  server.send(200, "text/plain", "Da chot moc FL 7cm!");
}

void handleCalibSideR7() {
  IrSnapshot liveIr = latestRawIrSnapshot();
  side_ref_FR = constrain(liveIr.frontRight, 0, 4095);
  saveRuntimeParams();
  server.send(200, "text/plain", "Da chot moc FR 7cm!");
}

void handleCmd() {
  if (server.hasArg("val")) {
    String command = server.arg("val");
    if (command == "START") {
      requestedState = PID_RUN;
      stateChangeRequested = true;
    } else if (command == "ONE_CELL") {
      requestedState = PID_RUN_ONE_CELL;
      stateChangeRequested = true;
    } else if (command == "STOP") {
      requestedState = IDLE;
      stateChangeRequested = true;
    } else if (command == "TEST_L") {
      requestedState = TEST_L;
      stateChangeRequested = true;
    } else if (command == "TEST_R") {
      requestedState = TEST_R;
      stateChangeRequested = true;
    }
  }
  server.send(200, "text/plain", "OK");
}

void handleData() {
  IrSnapshot liveIr = latestRawIrSnapshot();
  char jsonBuf[1280];
  snprintf(jsonBuf, sizeof(jsonBuf),
           "{\"irL\":%d,\"irFL\":%d,\"irFR\":%d,\"irR\":%d,"
           "\"dL\":%d,\"dFL\":%d,\"dFR\":%d,\"dR\":%d,"
           "\"encL\":%ld,\"encR\":%ld,"
           "\"pwmL\":%d,\"pwmR\":%d,\"state\":%d,"
           "\"maxFL\":%d,\"maxFR\":%d,"
           "\"sideL\":%d,\"sideFL\":%d,\"sideFR\":%d,\"sideR\":%d,"
           "\"errFL\":%d,\"errFR\":%d,\"steerIR\":%d,\"totalSteer\":%d,"
           "\"offUp\":%d,\"offLow\":%d,\"deadband\":%d,\"base\":%d,"
           "\"kpL\":%.3f,\"kiL\":%.3f,\"kdL\":%.3f,"
           "\"kpR\":%.3f,\"kiR\":%.3f,\"kdR\":%.3f,"
           "\"accel\":%.2f,\"vmax\":%.1f,\"vmin\":%.1f,"
           "\"ramp\":%d,\"gyro\":%.1f,\"irGain\":%.2f,\"pwmMin\":%d,\"pwmMax\":%d,"
           "\"xungO\":%d}",
           liveIr.left, liveIr.frontLeft, liveIr.frontRight, liveIr.right,
           liveIr.left - min_IR[SIDX(S_L)],
           liveIr.frontLeft - min_IR[SIDX(S_FL)],
           liveIr.frontRight - min_IR[SIDX(S_FR)],
           liveIr.right - min_IR[SIDX(S_R)], pulseL, pulseR, pwmL, pwmR,
           (int)carState, max_IR[SIDX(S_FL)], max_IR[SIDX(S_FR)],
           side_ref_L, side_ref_FL, side_ref_FR, side_ref_R,
           debugSideErrorL, debugSideErrorR, debugSteerIR, debugTotalSteer,
           offset_upper, offset_lower,
           ir_deadband, base_pwm, Kp_L, Ki_L, Kd_L, Kp_R, Ki_R, Kd_R,
           accel_rate, max_vel, min_vel, ramp_rate, k_gyro, k_ir, Turn_Min,
           Turn_Max, pulses_per_cell);
  server.send(200, "application/json", jsonBuf);
}

void webTask(void *pvParameters) {
  for (;;) {
    dnsServer.processNextRequest();
    server.handleClient();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void setupWebServer() {
  loadRuntimeParams();
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/calibHigh", handleCalibHigh);
  server.on("/calibLow", handleCalibLow);
  server.on("/calibSide7", handleCalibSide7);
  server.on("/calibSideL7", handleCalibSideL7);
  server.on("/calibSideR7", handleCalibSideR7);
  server.on("/calibSide5", handleCalibSide7);
  server.on("/cmd", handleCmd);
  server.on("/data", handleData);

  server.onNotFound([]() {
    server.sendHeader("Location",
                      String("http://") + server.client().localIP().toString(),
                      true);
    server.send(302, "text/plain", "");
  });
  server.begin();
  
  xTaskCreatePinnedToCore(webTask, "WebTask", 8192, NULL, 1, NULL, 0);
}
