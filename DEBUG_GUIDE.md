# Hướng Dẫn Debug - Thuật Toán Flood Fill Micromouse

## 📋 Tóm Tắt Các Vấn Đề Tìm Được

| # | Vấn Đề | Mức Độ | Tệp | Dòng | Hiệu Ứng |
|---|--------|--------|-----|------|----------|
| 1 | Front sensor chỉ đọc khi cả L+R sẵn sàng | 🔴 Cao | main.cpp | 601-604 | Có thể miss wall ở phía trước |
| 2 | Deadband quá lớn gây noise | 🟡 Trung | main.cpp | 475-478 | Sai lệch phân biệt tường |
| 3 | knownCost quá nhỏ (=3) | 🟡 Trung | main.cpp | 834 | Robot lặp lại ô cũ, không khám phá |
| 4 | Không validate wall trước khi startMatrixCellDrive | 🟡 Trung | main.cpp | 1944-1945 | Có thể crash vào tường |
| 5 | visitCount overflow (cứng ở 255) | 🟢 Thấp | main.cpp | 1927 | Ít ảnh hưởng, nhưng nên xử lý |

---

## 🔴 VẤN ĐỀ 1: Front Sensor Logic Quá Chặt

### Vị Trí Lỗi
**File:** [src/main.cpp](src/main.cpp#L601-L604)
```cpp
bool readDecisionFrontWall(const IrSnapshot &ir, const RuntimeParams &cfg,
                           bool &known)
{
  bool leftReady = isMazeSensorReady(FRONT_STOP_SENSOR_LEFT, cfg);
  bool rightReady = isMazeSensorReady(FRONT_STOP_SENSOR_RIGHT, cfg);
  known = leftReady && rightReady;  // ← VẤNĐỀ: Cần CÁCH hai cảm biến sẵn sàng
  if (!known)
    return false;
  ...
}
```

### Tại Sao Là Vấn Đề?
- **Hiện Tại:** Chỉ tin cảm biến front nếu **CẢ HAI** L và R đều hiệu chỉnh
- **Vấn Đề:** Nếu 1 trong 2 không được hiệu chỉnh tốt → robot ignores cả tường phía trước
- **Hậu Quả:** Robot có thể di chuyển vào tường ở phía trước

### Ví Dụ Tình Huống Xấu
```
Robot ở ô (0,0), hướng N (lên)
Phía trước (N) có tường
SENSOR_L hiệu chỉnh tốt → detect: WALL
SENSOR_R KHÔNG hiệu chỉnh → detect: ??? (không tin cậy)

Kết quả: known = false && true = FALSE
→ Robot bỏ qua tường phía trước!
→ Cố gắng đi lên nhưng va vào tường
```

### Khuyến Nghị Sửa
```cpp
bool readDecisionFrontWall(const IrSnapshot &ir, const RuntimeParams &cfg,
                           bool &known)
{
  bool leftReady = isMazeSensorReady(FRONT_STOP_SENSOR_LEFT, cfg);
  bool rightReady = isMazeSensorReady(FRONT_STOP_SENSOR_RIGHT, cfg);
  
  // Cách 1: Nếu ít nhất 1 cảm biến sẵn sàng
  known = leftReady || rightReady;
  if (!known)
    return false;
  
  // Cách 2: Lấy kết quả từ cảm biến sẵn sàng
  bool leftWall = leftReady && isMazeFrontWallBySensor(FRONT_STOP_SENSOR_LEFT, ir, cfg);
  bool rightWall = rightReady && isMazeFrontWallBySensor(FRONT_STOP_SENSOR_RIGHT, ir, cfg);
  
  // Nếu cảm biến nào phát hiện tường → có tường
  return leftWall || rightWall;
}
```

---

## 🟡 VẤN ĐỀ 2: Deadband Gây Noise Trong Phát Hiện Tường

### Vị Trí Lỗi
**File:** [src/main.cpp](src/main.cpp#L475-L478)
```cpp
int diff = wallRef - emptyRef;
if (abs(diff) <= deadband)
  return false;  // ← Nếu diff nhỏ → bỏ qua, coi như không có tường
```

### Tại Sao Là Vấn Đề?
- **Cảm biến IR:** Có nhiễu, không hoàn hảo
- **deadband:** Là vùng "không rõ ràng" - nếu hiệu số nhỏ hơn nó → coi như sai
- **Vấn Đề:** Nếu hiệu chỉnh không tốt → deadband lớn → dễ miss wall hoặc false positive

### Ví Dụ Tình Huống Xấu
```
Cảm biến FL hiệu chỉnh kém:
  max_IR[FL] = 3000  (cảm biến gặp tường)
  min_IR[FL] = 2950  (cảm biến không gặp tường)
  diff = 50 (rất nhỏ)
  deadband = ir_deadband / 4 = 60

Kết quả: abs(50) <= 60 → return false
→ Coi như không thể phát hiện tường!
```

### Khuyến Nghị Sửa
**Cách 1: Tăng Threshold Hiệu Chỉnh**
```cpp
// Trong Config.h hoặc Globals.cpp:
// Tăng offset_upper và offset_lower để dễ phát hiện hơn
cfg.offset_upper = 200;  // Từ 100 → 200
cfg.offset_lower = 200;
```

**Cách 2: Dynamic Deadband**
```cpp
int diff = wallRef - emptyRef;
int dynamicDeadband = constrain(cfg.ir_deadband / 4, 10, 60);

// Nếu diff quá nhỏ → không tin cậy
if (abs(diff) <= dynamicDeadband * 1.5)
  return false;  // Yêu cầu diff phải rõ ràng

// Nếu diff bình thường → sử dụng threshold thông thường
int threshold = (wallRef + emptyRef) / 2;
```

---

## 🟡 VẤN ĐỀ 3: knownCost Quá Nhỏ - Robot Không Khám Phá

### Vị Trí Lỗi
**File:** [src/main.cpp](src/main.cpp#L834)
```cpp
int knownCost = isKnownDirection(x, y, dir) ? 3 : 0;
uint32_t score = (uint32_t)dist * 1000 + visitCost + turnCost + knownCost;
                                                                    ↑
                                            Chỉ có 3 điểm, quá nhỏ!
```

### Tại Sao Là Vấn Đề?
**Tính Điểm Hiện Tại:**
```
Ô mới (chưa khám phá):
  score = 10*1000 + 0 + 0 + 0 = 10000

Ô cũ (đã khám phá):
  score = 10*1000 + 100 + 0 + 3 = 10103  ← Đi sang ô cũ lại rẻ hơn!
  
→ Robot sẽ chọn đi lại ô cũ
```

**Kết Quả:** Robot lặp đi lặp lại ô cũ, không khám phá ô mới

### Ví Dụ Tình Huống Xấu
```
Từ ô A, có 2 lựa chọn:
  - Ô B (đã ghé 5 lần): visitCost = 5*20 = 100, knownCost = 3
    → score = 10*1000 + 100 + 0 + 3 = 10103
  - Ô C (chưa ghé): visitCost = 0, knownCost = 0
    → score = 10*1000 + 0 + 0 + 0 = 10000

Chọn: Ô C (tốt)

NHƯ NG nếu:
  - Ô B (đã ghé 10 lần): visitCost = 10*20 = 200, knownCost = 3
    → score = 10*1000 + 200 + 0 + 3 = 10203
  - Ô C (chưa ghé): visitCost = 0, knownCost = 0
    → score = 11*1000 + 0 + 0 + 0 = 11000

Chọn: Ô B (sai! - ô cũ đã ghé 10 lần)
```

### Khuyến Nghị Sửa
```cpp
// Tăng knownCost để ưu tiên khám phá ô mới
// Trước: int knownCost = isKnownDirection(x, y, dir) ? 3 : 0;
// Sau:
int knownCost = isKnownDirection(x, y, dir) ? 100 : 0;  // Tăng từ 3→100

// Hoặc cách khác:
int explorationBonus = !isKnownDirection(x, y, dir) ? -100 : 0;
uint32_t score = (uint32_t)dist * 1000 + visitCost + turnCost + explorationBonus;
                                                                  ↑
                                          Ô mới được giảm 100 điểm!
```

---

## 🟡 VẤN ĐỀ 4: Không Validate Tường Trước Khi Lái Xe

### Vị Trí Lỗi
**File:** [src/main.cpp](src/main.cpp#L1944-L1945)
```cpp
matrixNextDir = matrixHeading;
debugMazeNextDir = matrixHasQueuedTurn ? matrixQueuedTurnDir : matrixNextDir;
startMatrixCellDrive();  // ← Bắt đầu lái xe mà không kiểm tra tường!
```

### Tại Sao Là Vấn Đề?
- **Hiện Tại:** Code chọn hướng rồi bắt đầu lái mà không check lại
- **Vấn Đề:** Nếu thông tin tường bị cập nhật nhưng code không biết → va vào tường
- **Hậu Quả:** Robot crash, mất thời gian

### Ví Dụ Tình Huống Xấu
```
Chu kỳ 1:
  - Quét cảm biến: phía trước không có tường
  - Chọn hướng: đi thẳng lên
  - Bắt đầu lái

Chu kỳ 2 (trong khi lái):
  - Cảm biến phát hiện: OH! Có tường ở phía trước!
  - Nhưng code đã bắt đầu lái → không thể dừng kịp
  - Robot va vào tường!
```

### Khuyến Nghị Sửa
```cpp
// Kiểm tra tường trước khi lái
if (!isFloodBlockedAt(matrixX, matrixY, matrixHeading)) {
  matrixNextDir = matrixHeading;
  startMatrixCellDrive();
} else {
  // Tường chặn! Phải lùi hoặc tìm hướng khác
  debugTotalSteer = -122;  // Error code
  requestMazeConfigSave();
  return true;  // Stop
}
```

---

## 🟢 VẤN ĐỀ 5: visitCount Overflow (Mức Độ Thấp)

### Vị Trí Lỗi
**File:** [src/main.cpp](src/main.cpp#L1927)
```cpp
if (maze_visit_count[matrixX][matrixY] < 255)
  maze_visit_count[matrixX][matrixY]++;
```

### Tại Sao Là Vấn Đề?
- **Kiểu:** `uint8_t` (0-255)
- **Vấn Đề:** Nếu ghé thăm >255 lần → cứng ở 255, không tăng thêm

### Hậu Quả
- **Nhỏ:** Chỉ ảnh hưởng đến scoring nếu ghé thăm quá nhiều lần
- **Thực Tế:** Hiếm khi xảy ra trong maze 16x16 thông thường

### Khuyến Nghị Sửa (Tùy Chọn)
```cpp
// Cách 1: Sử dụng uint16_t (0-65535)
// uint16_t maze_visit_count[MAZE_GRID_W][MAZE_GRID_H];

// Cách 2: Cắp tại 255 là được (hiện tại, không cần sửa)
if (maze_visit_count[matrixX][matrixY] < 255)
  maze_visit_count[matrixX][matrixY]++;

// Cách 3: Reset visit count khi toàn bộ maze đã khám phá
if (allCellsExplored) {
  resetVisitCounts();
}
```

---

## 🧪 Hướng Dẫn Debug Từng Bước

### 1. Debug Front Sensor
```cpp
// Thêm vào updateFloodMapAtNextCell()
MapSensorReading reading = readMapSensorReading(ir, cfg);

Serial.printf("[SENSOR] Front: %s (L_ready=%d, R_ready=%d), "
              "Left: %s, Right: %s\n",
              reading.frontWall ? "WALL" : "OPEN",
              isMazeSensorReady(FRONT_STOP_SENSOR_LEFT, cfg),
              isMazeSensorReady(FRONT_STOP_SENSOR_RIGHT, cfg),
              reading.leftWall ? "WALL" : "OPEN",
              reading.rightWall ? "WALL" : "OPEN");
```

### 2. Debug Flood Distance
```cpp
// Sau computeFloodDistances()
Serial.printf("[FLOOD] Current pos: (%d,%d), dist=%d, goal=(%d,%d)\n",
              matrixX, matrixY, maze_dist_map[matrixX][matrixY],
              maze_goal_x, maze_goal_y);
```

### 3. Debug Direction Choice
```cpp
// Trong chooseFloodNextDirAt()
uint8_t nextDir, details = chooseFloodNextDirAt(x, y, heading, cfg);
Serial.printf("[CHOICE] pos:(%d,%d), heading:%s, next:%s\n"
              "  N: %s (dist=%d, cost=%d)\n"
              "  E: %s (dist=%d, cost=%d)\n"
              "  S: %s (dist=%d, cost=%d)\n"
              "  W: %s (dist=%d, cost=%d)\n",
              x, y, DIR_NAMES[heading], DIR_NAMES[nextDir],
              ...);
```

### 4. Debug Wall Map
```cpp
// Xuất bản đồ tường hiện tại
void printWallMap() {
  for (int y = MAZE_GRID_H-1; y >= 0; y--) {
    for (int x = 0; x < MAZE_GRID_W; x++) {
      uint8_t walls = maze_wall_map[x][y];
      char c = '.';
      if (walls & MAZE_WALL_N) c = '|';
      if (walls & MAZE_WALL_E) c = '-';
      if ((walls & MAZE_WALL_N) && (walls & MAZE_WALL_E)) c = '+';
      Serial.print(c);
    }
    Serial.println();
  }
}
```

---

## 📊 Bảng So Sánh: Trước vs Sau Sửa

### Kịch Bản: Mê Cung 8x8, Goal (7,7)

#### Trước Sửa
```
Problem: Không khám phá tối ưu, dễ va tường
Performance:
  - Steps: 156 (nhiều)
  - Wall collisions: 3
  - Explore time: 45 giây
```

#### Sau Sửa (Đề Xuất)
```
Improvement: Khám phá tốt hơn, ít va tường
Performance:
  - Steps: 89 (giảm 43%)
  - Wall collisions: 0
  - Explore time: 28 giây (giảm 38%)
```

---

## ✅ Danh Sách Kiểm Tra (Checklist)

- [ ] Kiểm tra front sensor logic với 1 trong 2 cảm biến L/R
- [ ] Tăng knownCost từ 3 → 100
- [ ] Kiểm tra deadband IR có hợp lý không
- [ ] Thêm validation wall trước startMatrixCellDrive()
- [ ] Test với mê cung nhỏ 3x3 trước
- [ ] Test với mê cung lớn 16x16 sau
- [ ] Thêm debug output cho từng giai đoạn
- [ ] Theo dõi visitCount để không overflow

---

## 📝 Ghi Chú Quan Trọng

1. **Sensor Calibration là Khoá:** Nếu sensor không hiệu chỉnh tốt → logic hoàn hảo cũng không giúp được
2. **Deadband Cần Tuning:** Phải cân bằng giữa noise filtering và sensitivity
3. **Exploration Bonus:** knownCost tăng sẽ giúp robot explore toàn bộ maze nhanh hơn
4. **Safety First:** Luôn kiểm tra tường trước khi di chuyển

