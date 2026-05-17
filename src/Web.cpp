#include "Web.h"
#include "Globals.h"
#include "IMU.h"
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
  k_gyro = max(0.0f, k_gyro);
  k_ir = max(0.0f, k_ir);
  wall_steer_limit = constrain(wall_steer_limit, 4, max(4, Turn_Max - Turn_Min));
  wheel_trim_L = constrain(wheel_trim_L, -60, 60);
  wheel_trim_R = constrain(wheel_trim_R, -60, 60);
  ir_deadband = max(0, ir_deadband);
  offset_upper = max(0, offset_upper);
  offset_lower = max(0, offset_lower);
  pulses_per_cell = max(1, pulses_per_cell);
  front_stop_early_margin = constrain(front_stop_early_margin, 0, 4095);
  maze_dead_end_backup_pulses = constrain(maze_dead_end_backup_pulses, 0, 5000);
  maze_dead_end_backup_pwm = constrain(maze_dead_end_backup_pwm, Turn_Min, Turn_Max);
  maze_turn_late_pulses = constrain(maze_turn_late_pulses, 0, 5000);
  maze_turn_angle_deg = constrain(maze_turn_angle_deg, 45.0f, 90.0f);
  point_turn_backup_pulses = constrain(point_turn_backup_pulses, 0, 5000);
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
  wall_steer_limit = settings.getInt("wallMax", wall_steer_limit);
  wheel_trim_L = settings.getInt("trimL", wheel_trim_L);
  wheel_trim_R = settings.getInt("trimR", wheel_trim_R);
  Turn_Min = settings.getInt("pwmMin", Turn_Min);
  Turn_Max = settings.getInt("pwmMax", Turn_Max);
  pulses_per_cell = settings.getInt("xungO", pulses_per_cell);
  front_stop_early_margin = settings.getInt("frontMg", front_stop_early_margin);
  maze_dead_end_backup_pulses = settings.getInt("deadBack", maze_dead_end_backup_pulses);
  maze_dead_end_backup_pwm = settings.getInt("backPwm", maze_dead_end_backup_pwm);
  maze_turn_late_pulses = settings.getInt("turnLate", maze_turn_late_pulses);
  maze_turn_angle_deg = settings.getFloat("turnDeg", maze_turn_angle_deg);
  point_turn_backup_pulses = settings.getInt("turnBack", point_turn_backup_pulses);
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
  settings.putInt("wallMax", wall_steer_limit);
  settings.putInt("trimL", wheel_trim_L);
  settings.putInt("trimR", wheel_trim_R);
  settings.putInt("pwmMin", Turn_Min);
  settings.putInt("pwmMax", Turn_Max);
  settings.putInt("xungO", pulses_per_cell);
  settings.putInt("frontMg", front_stop_early_margin);
  settings.putInt("deadBack", maze_dead_end_backup_pulses);
  settings.putInt("backPwm", maze_dead_end_backup_pwm);
  settings.putInt("turnLate", maze_turn_late_pulses);
  settings.putFloat("turnDeg", maze_turn_angle_deg);
  settings.putInt("turnBack", point_turn_backup_pulses);
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

uint8_t hexToNibble(char c) {
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'A' && c <= 'F')
    return 10 + c - 'A';
  if (c >= 'a' && c <= 'f')
    return 10 + c - 'a';
  return 0;
}

char nibbleToHex(uint8_t value) {
  value &= 0x0F;
  return (value < 10) ? ('0' + value) : ('A' + value - 10);
}

uint8_t pathCharToValue(char c) {
  if (c == 'N' || c == 'n')
    return MAZE_PATH_N;
  if (c == 'E' || c == 'e')
    return MAZE_PATH_E;
  if (c == 'S' || c == 's')
    return MAZE_PATH_S;
  if (c == 'W' || c == 'w')
    return MAZE_PATH_W;
  if (c == 'X' || c == 'x')
    return MAZE_PATH_STOP;
  return MAZE_PATH_EMPTY;
}

char pathValueToChar(uint8_t value) {
  switch (value) {
  case MAZE_PATH_N:
    return 'N';
  case MAZE_PATH_E:
    return 'E';
  case MAZE_PATH_S:
    return 'S';
  case MAZE_PATH_W:
    return 'W';
  case MAZE_PATH_STOP:
    return 'X';
  default:
    return '.';
  }
}

String serializeMazeWalls() {
  String out;
  out.reserve(MAZE_GRID_CELLS);
  for (int y = 0; y < MAZE_GRID_H; y++) {
    for (int x = 0; x < MAZE_GRID_W; x++) {
      out += nibbleToHex(maze_wall_map[x][y]);
    }
  }
  return out;
}

String serializeMazePath() {
  String out;
  out.reserve(MAZE_GRID_CELLS);
  for (int y = 0; y < MAZE_GRID_H; y++) {
    for (int x = 0; x < MAZE_GRID_W; x++) {
      out += pathValueToChar(maze_path_map[x][y]);
    }
  }
  return out;
}

void applyMazeStrings(const String &walls, const String &path) {
  for (int y = 0; y < MAZE_GRID_H; y++) {
    for (int x = 0; x < MAZE_GRID_W; x++) {
      int idx = y * MAZE_GRID_W + x;
      maze_wall_map[x][y] =
          (idx < walls.length()) ? (hexToNibble(walls[idx]) & 0x0F) : 0;
      maze_path_map[x][y] =
          (idx < path.length()) ? pathCharToValue(path[idx]) : MAZE_PATH_EMPTY;
    }
  }
}

void loadMazeConfig() {
  resetMazeMaps();
  settings.begin("robot_pid", true);
  String walls = settings.getString("mazeWalls", "");
  String path = settings.getString("mazePath", "");
  maze_start_x = settings.getInt("mazeSX", maze_start_x);
  maze_start_y = settings.getInt("mazeSY", maze_start_y);
  maze_start_heading = settings.getInt("mazeHead", maze_start_heading);
  settings.end();

  maze_start_x = constrain(maze_start_x, 0, MAZE_GRID_W - 1);
  maze_start_y = constrain(maze_start_y, 0, MAZE_GRID_H - 1);
  if (maze_start_heading < MAZE_PATH_N || maze_start_heading > MAZE_PATH_W)
    maze_start_heading = MAZE_PATH_N;
  applyMazeStrings(walls, path);
}

void saveMazeConfig() {
  maze_start_x = constrain(maze_start_x, 0, MAZE_GRID_W - 1);
  maze_start_y = constrain(maze_start_y, 0, MAZE_GRID_H - 1);
  if (maze_start_heading < MAZE_PATH_N || maze_start_heading > MAZE_PATH_W)
    maze_start_heading = MAZE_PATH_N;

  settings.begin("robot_pid", false);
  settings.putString("mazeWalls", serializeMazeWalls());
  settings.putString("mazePath", serializeMazePath());
  settings.putInt("mazeSX", maze_start_x);
  settings.putInt("mazeSY", maze_start_y);
  settings.putInt("mazeHead", maze_start_heading);
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
  select { width: 56px; padding: 3px; border: 1px solid #ccc; border-radius: 3px; font-weight: bold; }
  .maze-tools { display: grid; grid-template-columns: repeat(4, 1fr); gap: 4px; margin-bottom: 6px; }
  .maze-tools .btn { padding: 6px 2px; margin-bottom: 0; background: #607d8b; }
  .maze-tools .active { background: #2c3e50; box-shadow: inset 0 0 0 2px #fff; }
  .maze-start { display: grid; grid-template-columns: repeat(3, 1fr); gap: 6px; margin-bottom: 6px; }
  .maze-grid { display: grid; grid-template-columns: repeat(16, minmax(18px, 1fr)); gap: 1px; background: #b0bec5; border: 1px solid #78909c; padding: 2px; margin-bottom: 6px; user-select: none; }
  .maze-cell { aspect-ratio: 1 / 1; background: #fff; color: #263238; font-size: 10px; font-weight: bold; display: flex; align-items: center; justify-content: center; cursor: pointer; border: 1px solid #d7dde1; box-sizing: border-box; }
  .maze-cell.start { background: #d5f5e3; }
  .maze-cell.stop { background: #fadbd8; }
  .maze-cell:hover { background: #e3f2fd; }
  .maze-help { font-size: 10px; color: #546e7a; margin-bottom: 5px; }
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
        <h4>MPU GOC</h4>
        <div class="val-box">
          <div class="val-item">Yaw: <span id="yaw" class="raw">0.00</span></div>
          <div class="val-item">Target: <span id="target_yaw" class="raw">0.00</span></div>
          <div class="val-item">Err: <span id="yaw_err" class="delta">0.00</span></div>
          <div class="val-item">Steer: <span id="total_steer" class="delta">0</span></div>
          <div class="val-item">GyroZ Raw: <span id="gyro_z_raw" class="raw">0.000</span></div>
          <div class="val-item">GyroZ Corr: <span id="gyro_z_corr" class="raw">0.000</span></div>
          <div class="val-item">BiasZ: <span id="gyro_z_bias" class="delta">0.000</span></div>
          <div class="val-item">Drift/min: <span id="yaw_drift" class="delta">0.00</span></div>
          <div class="val-item">Temp C: <span id="mpu_temp" class="raw">0.0</span></div>
          <div class="val-item">AutoBias: <span id="auto_bias" class="raw">OFF</span></div>
          <div class="val-item">Still ms: <span id="auto_bias_ms" class="raw">0</span></div>
          <div class="val-item">AB Reason: <span id="auto_bias_reason" class="delta">WAIT</span></div>
        </div>
        <button class="btn" style="background:#2c3e50;" onclick="rezeroMpu()">REZERO MPU</button>
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
        <label>Bam Tuong <input type="number" id="ir_gain" step="0.01" value="0.1"></label>
        <label>Wall Max <input type="number" id="wall_steer" step="1" value="22"></label>
        <label>Trim L <input type="number" id="trim_l" step="1" value="0"></label>
        <label>Trim R <input type="number" id="trim_r" step="1" value="0"></label>
      </div>
      <div class="group">
        <h4>KHUNG GAM</h4>
        <label>Ramp <input type="number" id="ramp" step="1" value="10"></label>
        <label>Gyro <input type="number" id="gyro" step="0.1" value="6.0"></label>
        <label>PWM Min <input type="number" id="pwm_min" step="1" value="35"></label>
        <label>PWM Max <input type="number" id="pwm_max" step="1" value="100"></label>
        <label>Xung/20cm <input type="number" id="xung_o" step="1" value="980"></label>
        <label>Front Margin <input type="number" id="front_margin" step="1" value="800"></label>
        <label>Tre Re <input type="number" id="turn_late" step="1" value="120"></label>
        <label>Goc Re <input type="number" id="turn_deg" step="1" value="78"></label>
        <label>Lui Sau Re <input type="number" id="turn_back" step="1" value="80"></label>
        <label>Lui Ngo Cut <input type="number" id="dead_back" step="1" value="980"></label>
        <label>PWM Lui <input type="number" id="backup_pwm" step="1" value="55"></label>
      </div>
    </div>
    
    <div class="grid-2">
      <button class="btn btn-calib-h" onclick="calib('High').then(r=>r.text())">🎯 SET TƯỜNG (RAW - OFF)</button>
      <button class="btn btn-calib-l" onclick="calib('Low').then(r=>r.text())">🌌 SET TRỐNG (RAW + OFF)</button>
    </div>
    <div class="grid-2">
      <button class="btn" style="background:#16a085;" onclick="calib('SideL7').then(r=>r.text())">SET TUONG TRAI</button>
      <button class="btn" style="background:#16a085;" onclick="calib('SideR7').then(r=>r.text())">SET TUONG PHAI</button>
    </div>
    <button class="btn" style="background:#1abc9c;" onclick="calib('Center').then(r=>r.text())">SET GIUA O</button>
    <button class="btn btn-save" onclick="saveAll()">💾 LƯU THÔNG SỐ BĂM XUNG</button>
    
    <div class="grid-2">
      <button class="btn" style="background:#8e44ad;" onclick="fetch('/cmd?val=TEST_L').then(r=>r.text())">TEST MOTOR TRAI</button>
      <button class="btn" style="background:#d35400;" onclick="fetch('/cmd?val=TEST_R').then(r=>r.text())">TEST MOTOR PHAI</button>
    </div>
    <div class="grid-2">
      <button class="btn" style="background:#6c3483;" onclick="fetch('/cmd?val=TEST_TURN_L').then(r=>r.text())">TEST QUAY TRAI</button>
      <button class="btn" style="background:#ba4a00;" onclick="fetch('/cmd?val=TEST_TURN_R').then(r=>r.text())">TEST QUAY PHAI</button>
    </div>
    <button class="btn" style="background:#7f8c8d;" onclick="fetch('/cmd?val=BACK_ONE_CELL').then(r=>r.text())">LUI 1 O</button>

    <div class="group">
      <h4>MA TRAN DUONG DI 16x16</h4>
      <div class="maze-help">O (0,0) nam o goc duoi-trai. Bam N/E/S/W de dat huong di tiep, X la dich/dung, . la xoa o. Wall N/E/S/W de bat/tat tuong cua o.</div>
      <div class="maze-start">
        <label>X0 <input type="number" id="maze_sx" min="0" max="15" value="0"></label>
        <label>Y0 <input type="number" id="maze_sy" min="0" max="15" value="0"></label>
        <label>Head <select id="maze_head"><option>N</option><option>E</option><option>S</option><option>W</option></select></label>
      </div>
      <div class="maze-tools">
        <button class="btn maze-tool active" data-tool="N" onclick="setMazeTool('N')">N</button>
        <button class="btn maze-tool" data-tool="E" onclick="setMazeTool('E')">E</button>
        <button class="btn maze-tool" data-tool="S" onclick="setMazeTool('S')">S</button>
        <button class="btn maze-tool" data-tool="W" onclick="setMazeTool('W')">W</button>
        <button class="btn maze-tool" data-tool="X" onclick="setMazeTool('X')">X</button>
        <button class="btn maze-tool" data-tool="." onclick="setMazeTool('.')">.</button>
        <button class="btn maze-tool" data-tool="wallN" onclick="setMazeTool('wallN')">Wall N</button>
        <button class="btn maze-tool" data-tool="wallE" onclick="setMazeTool('wallE')">Wall E</button>
        <button class="btn maze-tool" data-tool="wallS" onclick="setMazeTool('wallS')">Wall S</button>
        <button class="btn maze-tool" data-tool="wallW" onclick="setMazeTool('wallW')">Wall W</button>
      </div>
      <div id="mazeGrid" class="maze-grid"></div>
      <div class="grid-2">
        <button class="btn" style="background:#34495e;" onclick="saveMazeEditor()">LUU MA TRAN</button>
        <button class="btn btn-run" style="background:#00a085;" onclick="fetch('/cmd?val=MATRIX_RUN')">CHAY MA TRAN</button>
      </div>
    </div>

    <div class="grid-btn">
      <button class="btn btn-run" onclick="fetch('/cmd?val=START')">CHAY LIEN TUC</button>
      <button class="btn btn-run" style="background:#27ae60;" onclick="fetch('/cmd?val=ONE_CELL')">DI THANG 1 O</button>
      <button class="btn btn-run" style="background:#2980b9;" onclick="fetch('/cmd?val=MAZE_RIGHT')">GIAI ME CUNG PHAI</button>
      <button class="btn btn-run" style="background:#2471a3;" onclick="fetch('/cmd?val=MAZE_RIGHT_CELL')">GIAI TUNG O PHAI</button>
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
    ramp: 'ramp', gyro: 'gyro', irGain: 'ir_gain', wallMax: 'wall_steer',
    trimL: 'trim_l', trimR: 'trim_r', pwmMin: 'pwm_min', pwmMax: 'pwm_max',
    xungO: 'xung_o', frontMargin: 'front_margin', deadBack: 'dead_back',
    backupPwm: 'backup_pwm', turnLate: 'turn_late', turnDeg: 'turn_deg',
    turnBack: 'turn_back'
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

  let mazeW = 16, mazeH = 16, mazeWalls = '', mazePath = '', mazeTool = 'N';
  const wallBits = { W: 1, E: 2, S: 4, N: 8 };
  const oppositeWall = { W: 'E', E: 'W', S: 'N', N: 'S' };
  const pathDisplay = { N: '^', E: '>', S: 'v', W: '<', X: 'X', '.': '' };

  function mazeIndex(x, y) { return y * mazeW + x; }
  function replaceAt(str, idx, ch) { return str.substring(0, idx) + ch + str.substring(idx + 1); }
  function wallNibble(idx) {
    let n = parseInt(mazeWalls[idx] || '0', 16);
    return isNaN(n) ? 0 : n;
  }
  function setWallNibble(idx, value) {
    mazeWalls = replaceAt(mazeWalls, idx, (value & 15).toString(16).toUpperCase());
  }
  function normalizeMazeEditor() {
    const cells = mazeW * mazeH;
    if(mazeWalls.length !== cells) mazeWalls = '0'.repeat(cells);
    if(mazePath.length !== cells) mazePath = '.'.repeat(cells);
  }
  function setMazeTool(tool) {
    mazeTool = tool;
    document.querySelectorAll('.maze-tool').forEach(btn => {
      btn.classList.toggle('active', btn.dataset.tool === tool);
    });
  }
  function neighborForWall(x, y, dir) {
    if(dir === 'N') return [x, y + 1];
    if(dir === 'E') return [x + 1, y];
    if(dir === 'S') return [x, y - 1];
    return [x - 1, y];
  }
  function toggleMazeWall(x, y, dir) {
    const idx = mazeIndex(x, y);
    const bit = wallBits[dir];
    const nextValue = wallNibble(idx) ^ bit;
    setWallNibble(idx, nextValue);

    const [nx, ny] = neighborForWall(x, y, dir);
    if(nx >= 0 && nx < mazeW && ny >= 0 && ny < mazeH) {
      const nidx = mazeIndex(nx, ny);
      setWallNibble(nidx, wallNibble(nidx) ^ wallBits[oppositeWall[dir]]);
    }
  }
  function clickMazeCell(x, y) {
    normalizeMazeEditor();
    if(mazeTool.startsWith('wall')) {
      toggleMazeWall(x, y, mazeTool.substring(4));
    } else {
      mazePath = replaceAt(mazePath, mazeIndex(x, y), mazeTool);
    }
    renderMazeGrid();
  }
  function renderMazeGrid() {
    normalizeMazeEditor();
    const grid = document.getElementById('mazeGrid');
    if(!grid) return;
    grid.innerHTML = '';
    const sx = parseInt(document.getElementById('maze_sx').value || '0', 10);
    const sy = parseInt(document.getElementById('maze_sy').value || '0', 10);
    for(let y = mazeH - 1; y >= 0; y--) {
      for(let x = 0; x < mazeW; x++) {
        const idx = mazeIndex(x, y);
        const cell = document.createElement('div');
        const walls = wallNibble(idx);
        const p = mazePath[idx] || '.';
        cell.className = 'maze-cell';
        if(x === sx && y === sy) cell.classList.add('start');
        if(p === 'X') cell.classList.add('stop');
        cell.textContent = pathDisplay[p] || '';
        cell.title = `(${x},${y}) path=${p} walls=${mazeWalls[idx] || '0'}`;
        cell.style.borderLeftWidth = (walls & wallBits.W) ? '3px' : '1px';
        cell.style.borderRightWidth = (walls & wallBits.E) ? '3px' : '1px';
        cell.style.borderBottomWidth = (walls & wallBits.S) ? '3px' : '1px';
        cell.style.borderTopWidth = (walls & wallBits.N) ? '3px' : '1px';
        cell.style.borderLeftColor = (walls & wallBits.W) ? '#263238' : '#d7dde1';
        cell.style.borderRightColor = (walls & wallBits.E) ? '#263238' : '#d7dde1';
        cell.style.borderBottomColor = (walls & wallBits.S) ? '#263238' : '#d7dde1';
        cell.style.borderTopColor = (walls & wallBits.N) ? '#263238' : '#d7dde1';
        cell.onclick = () => clickMazeCell(x, y);
        grid.appendChild(cell);
      }
    }
  }
  function loadMazeEditor() {
    fetch('/mazeData').then(r => r.json()).then(d => {
      mazeW = d.w || 16; mazeH = d.h || 16;
      mazeWalls = d.walls || ''; mazePath = d.path || '';
      document.getElementById('maze_sx').value = d.sx || 0;
      document.getElementById('maze_sy').value = d.sy || 0;
      document.getElementById('maze_head').value = d.head || 'N';
      normalizeMazeEditor();
      renderMazeGrid();
    }).catch(e => {
      normalizeMazeEditor();
      renderMazeGrid();
    });
  }
  function saveMazeEditor() {
    normalizeMazeEditor();
    const sx = encodeURIComponent(document.getElementById('maze_sx').value);
    const sy = encodeURIComponent(document.getElementById('maze_sy').value);
    const head = encodeURIComponent(document.getElementById('maze_head').value);
    fetch(`/mazeSet?walls=${mazeWalls}&path=${mazePath}&sx=${sx}&sy=${sy}&head=${head}`)
      .then(r => r.text().then(text => {
        document.getElementById('console').innerHTML = text;
        if(!r.ok) alert(text);
        return text;
      }));
  }
  document.getElementById('maze_sx').addEventListener('change', renderMazeGrid);
  document.getElementById('maze_sy').addEventListener('change', renderMazeGrid);

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
      "wallMax=" + val('wall_steer'),
      "trimL=" + val('trim_l'),
      "trimR=" + val('trim_r'),
      "pwmMin=" + val('pwm_min'),
      "pwmMax=" + val('pwm_max'),
      "xungO=" + val('xung_o'),
      "frontMargin=" + val('front_margin'),
      "turnLate=" + val('turn_late'),
      "turnDeg=" + val('turn_deg'),
      "turnBack=" + val('turn_back'),
      "deadBack=" + val('dead_back'),
      "backupPwm=" + val('backup_pwm')
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

  function rezeroMpu() {
    document.getElementById('console').innerHTML = 'Rezero MPU... keep robot still';
    return fetch('/imuRezero')
      .then(response => response.text().then(text => { alert(text); return response; }))
      .catch(err => alert("MPU rezero failed"));
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
          document.getElementById('yaw').innerText = d.yaw.toFixed(2);
          document.getElementById('target_yaw').innerText = d.targetYaw.toFixed(2);
          document.getElementById('yaw_err').innerText = d.yawErr.toFixed(2);
          document.getElementById('total_steer').innerText = d.totalSteer;
          document.getElementById('gyro_z_raw').innerText = d.gyroZRaw.toFixed(3);
          document.getElementById('gyro_z_corr').innerText = d.gyroZCorr.toFixed(3);
          document.getElementById('gyro_z_bias').innerText = d.gyroZBias.toFixed(3);
          document.getElementById('yaw_drift').innerText = d.yawDriftDpm.toFixed(2);
          document.getElementById('mpu_temp').innerText = d.mpuTemp.toFixed(1);
          document.getElementById('auto_bias').innerText = d.autoBias ? 'ON' : 'OFF';
          document.getElementById('auto_bias_ms').innerText = d.autoBiasMs;
          const abReasons = ['ON', 'WAIT', 'STATE', 'PWM', 'ENC', 'GYRO'];
          document.getElementById('auto_bias_reason').innerText = abReasons[d.autoBiasReason] || 'UNKNOWN';

          let spdL = lastViewEncL === null ? 0 : Math.abs(d.encL - lastViewEncL);
          let spdR = lastViewEncR === null ? 0 : Math.abs(d.encR - lastViewEncR);
          lastViewEncL = d.encL; lastViewEncR = d.encR;

          document.getElementById('console').innerHTML = `<span style="color:#c0392b; font-weight:bold;">ENC_L:</span> ${d.encL} | <span style="color:#2c3e50; font-weight:bold;">ENC_R:</span> ${d.encR} | SPD: ${spdL}/${spdR} | PWM: ${d.pwmL}/${d.pwmR} | CELL: ${d.mazeX},${d.mazeY} h${d.mazeHead} n${d.mazeNext} | YAW: ${d.yaw.toFixed(2)} / ${d.targetYaw.toFixed(2)} e${d.yawErr.toFixed(2)} | GZ: ${d.gyroZCorr.toFixed(3)} dps (${d.yawDriftDpm.toFixed(1)} dpm) | SIDE: FL ${d.irFL}/${d.sideFL} e${d.errFL} - FR ${d.irFR}/${d.sideFR} e${d.errFR} | STEER: ${d.steerIR}/${d.totalSteer} | STATE: ${d.state}`;
          
          setTimeout(fetchLoop, 100); 
        }).catch(e => { setTimeout(fetchLoop, 500); });
    } else { setTimeout(fetchLoop, 100); }
  }
  loadMazeEditor();
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
  if (server.hasArg("wallMax")) wall_steer_limit = server.arg("wallMax").toInt();
  if (server.hasArg("trimL")) wheel_trim_L = server.arg("trimL").toInt();
  if (server.hasArg("trimR")) wheel_trim_R = server.arg("trimR").toInt();
  if (server.hasArg("pwmMin")) Turn_Min = server.arg("pwmMin").toInt();
  if (server.hasArg("pwmMax")) Turn_Max = server.arg("pwmMax").toInt();
  if (server.hasArg("xungO")) pulses_per_cell = server.arg("xungO").toInt();
  if (server.hasArg("frontMargin")) front_stop_early_margin = server.arg("frontMargin").toInt();
  if (server.hasArg("turnLate")) maze_turn_late_pulses = server.arg("turnLate").toInt();
  if (server.hasArg("turnDeg")) maze_turn_angle_deg = server.arg("turnDeg").toFloat();
  if (server.hasArg("turnBack")) point_turn_backup_pulses = server.arg("turnBack").toInt();
  if (server.hasArg("deadBack")) maze_dead_end_backup_pulses = server.arg("deadBack").toInt();
  if (server.hasArg("backupPwm")) maze_dead_end_backup_pwm = server.arg("backupPwm").toInt();
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

void handleCalibSideL7() {
  IrSnapshot liveIr = latestRawIrSnapshot();
  side_ref_FL = constrain(liveIr.frontLeft, 0, 4095);
  saveRuntimeParams();
  server.send(200, "text/plain", "Da chot moc tuong trai!");
}

void handleCalibSideR7() {
  IrSnapshot liveIr = latestRawIrSnapshot();
  side_ref_FR = constrain(liveIr.frontRight, 0, 4095);
  saveRuntimeParams();
  server.send(200, "text/plain", "Da chot moc tuong phai!");
}

void handleCalibCenter() {
  IrSnapshot liveIr = latestRawIrSnapshot();
  side_ref_FL = constrain(liveIr.frontLeft, 0, 4095);
  side_ref_FR = constrain(liveIr.frontRight, 0, 4095);
  saveRuntimeParams();
  server.send(200, "text/plain", "Da chot moc giua o!");
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
    } else if (command == "MAZE_RIGHT") {
      requestedState = MAZE_RIGHT_HAND;
      stateChangeRequested = true;
    } else if (command == "MAZE_RIGHT_CELL") {
      requestedState = MAZE_RIGHT_HAND_CELL;
      stateChangeRequested = true;
    } else if (command == "MATRIX_RUN") {
      requestedState = MAZE_MATRIX_RUN;
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
    } else if (command == "TEST_TURN_L") {
      requestedState = TEST_TURN_L;
      stateChangeRequested = true;
    } else if (command == "TEST_TURN_R") {
      requestedState = TEST_TURN_R;
      stateChangeRequested = true;
    } else if (command == "BACK_ONE_CELL") {
      requestedState = TEST_BACK_ONE_CELL;
      stateChangeRequested = true;
    }
  }
  server.send(200, "text/plain", "OK");
}

void handleImuRezero() {
  requestedState = IDLE;
  stateChangeRequested = true;
  delay(80);

  bool ok = rezeroIMU(1500);
  if (ok) {
    server.send(200, "text/plain", "MPU rezero OK. Keep robot still for AutoBias.");
  } else {
    server.send(409, "text/plain", "MPU rezero is already running.");
  }
}

void handleMazeData() {
  String walls = serializeMazeWalls();
  String path = serializeMazePath();
  String json;
  json.reserve(MAZE_GRID_CELLS * 2 + 180);
  json += "{\"w\":";
  json += MAZE_GRID_W;
  json += ",\"h\":";
  json += MAZE_GRID_H;
  json += ",\"walls\":\"";
  json += walls;
  json += "\",\"path\":\"";
  json += path;
  json += "\",\"sx\":";
  json += maze_start_x;
  json += ",\"sy\":";
  json += maze_start_y;
  json += ",\"head\":\"";
  json += pathValueToChar((uint8_t)maze_start_heading);
  json += "\",\"x\":";
  json += debugMazeX;
  json += ",\"y\":";
  json += debugMazeY;
  json += ",\"heading\":";
  json += debugMazeHeading;
  json += ",\"next\":";
  json += debugMazeNextDir;
  json += "}";
  server.send(200, "application/json", json);
}

void handleMazeSet() {
  if (isRunning) {
    server.send(409, "text/plain", "Hay dung xe truoc khi luu ma tran.");
    return;
  }

  if (server.hasArg("walls") && server.hasArg("path")) {
    applyMazeStrings(server.arg("walls"), server.arg("path"));
  }
  if (server.hasArg("sx"))
    maze_start_x = server.arg("sx").toInt();
  if (server.hasArg("sy"))
    maze_start_y = server.arg("sy").toInt();
  if (server.hasArg("head") && server.arg("head").length() > 0)
    maze_start_heading = pathCharToValue(server.arg("head")[0]);

  saveMazeConfig();
  server.send(200, "text/plain", "Da luu ma tran duong di.");
}

void handleData() {
  IrSnapshot liveIr = latestRawIrSnapshot();
  float liveYaw = continuousYaw;
  float liveTargetYaw = target_yaw;
  float liveYawErr = liveYaw - liveTargetYaw;
  float liveGyroZRaw = debugGyroZRawDps;
  float liveGyroZCorr = debugGyroZCorrectedDps;
  float liveGyroZBias = debugGyroZBiasDps;
  float liveMpuTemp = debugMpuTempC;
  float liveYawDriftDpm = debugYawDriftDpm;
  int liveAutoBias = debugGyroZAutoBiasActive;
  uint32_t liveAutoBiasMs = debugGyroZAutoBiasStillMs;
  int liveAutoBiasReason = debugGyroZAutoBiasReason;
  char jsonBuf[2400];
  snprintf(jsonBuf, sizeof(jsonBuf),
           "{\"irL\":%d,\"irFL\":%d,\"irFR\":%d,\"irR\":%d,"
           "\"dL\":%d,\"dFL\":%d,\"dFR\":%d,\"dR\":%d,"
           "\"encL\":%ld,\"encR\":%ld,"
           "\"pwmL\":%d,\"pwmR\":%d,\"state\":%d,"
           "\"mazeX\":%d,\"mazeY\":%d,\"mazeHead\":%d,\"mazeNext\":%d,"
           "\"yaw\":%.2f,\"targetYaw\":%.2f,\"yawErr\":%.2f,"
           "\"gyroZRaw\":%.3f,\"gyroZCorr\":%.3f,\"gyroZBias\":%.3f,"
           "\"mpuTemp\":%.2f,\"yawDriftDpm\":%.2f,"
           "\"autoBias\":%d,\"autoBiasMs\":%lu,\"autoBiasReason\":%d,"
           "\"maxFL\":%d,\"maxFR\":%d,"
           "\"sideL\":%d,\"sideFL\":%d,\"sideFR\":%d,\"sideR\":%d,"
           "\"errFL\":%d,\"errFR\":%d,\"steerIR\":%d,\"totalSteer\":%d,"
           "\"offUp\":%d,\"offLow\":%d,\"deadband\":%d,\"base\":%d,"
           "\"kpL\":%.3f,\"kiL\":%.3f,\"kdL\":%.3f,"
           "\"kpR\":%.3f,\"kiR\":%.3f,\"kdR\":%.3f,"
           "\"accel\":%.2f,\"vmax\":%.1f,\"vmin\":%.1f,"
           "\"ramp\":%d,\"gyro\":%.1f,\"irGain\":%.2f,"
           "\"wallMax\":%d,\"trimL\":%d,\"trimR\":%d,"
           "\"pwmMin\":%d,\"pwmMax\":%d,"
           "\"xungO\":%d,\"frontMargin\":%d,\"turnLate\":%d,\"turnDeg\":%.1f,"
           "\"turnBack\":%d,"
           "\"deadBack\":%d,"
           "\"backupPwm\":%d}",
           liveIr.left, liveIr.frontLeft, liveIr.frontRight, liveIr.right,
           liveIr.left - min_IR[SIDX(S_L)],
           liveIr.frontLeft - min_IR[SIDX(S_FL)],
           liveIr.frontRight - min_IR[SIDX(S_FR)],
           liveIr.right - min_IR[SIDX(S_R)], pulseL, pulseR, pwmL, pwmR,
           (int)carState, debugMazeX, debugMazeY, debugMazeHeading,
           debugMazeNextDir, liveYaw, liveTargetYaw, liveYawErr,
           liveGyroZRaw, liveGyroZCorr, liveGyroZBias, liveMpuTemp,
           liveYawDriftDpm, liveAutoBias, (unsigned long)liveAutoBiasMs,
           liveAutoBiasReason,
           max_IR[SIDX(S_FL)], max_IR[SIDX(S_FR)],
           side_ref_L, side_ref_FL, side_ref_FR, side_ref_R,
           debugSideErrorL, debugSideErrorR, debugSteerIR, debugTotalSteer,
           offset_upper, offset_lower,
           ir_deadband, base_pwm, Kp_L, Ki_L, Kd_L, Kp_R, Ki_R, Kd_R,
           accel_rate, max_vel, min_vel, ramp_rate, k_gyro, k_ir,
           wall_steer_limit, wheel_trim_L, wheel_trim_R, Turn_Min, Turn_Max,
           pulses_per_cell, front_stop_early_margin, maze_turn_late_pulses,
           maze_turn_angle_deg, point_turn_backup_pulses,
           maze_dead_end_backup_pulses,
           maze_dead_end_backup_pwm);
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
  loadMazeConfig();
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/calibHigh", handleCalibHigh);
  server.on("/calibLow", handleCalibLow);
  server.on("/calibSideL7", handleCalibSideL7);
  server.on("/calibSideR7", handleCalibSideR7);
  server.on("/calibCenter", handleCalibCenter);
  server.on("/cmd", handleCmd);
  server.on("/imuRezero", handleImuRezero);
  server.on("/mazeData", handleMazeData);
  server.on("/mazeSet", handleMazeSet);
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
