# Phân Tích Thuật Toán Giải Mê Cung (Flood Fill)

## 1. Tóm Tắt Cấu Trúc Chung

Thuật toán của bạn sử dụng **Flood Fill** kết hợp với **Matrix-based Path Planning** để giải mê cung.

### Các Cảm Biến:
- **SENSOR_FL** (0): Hướng Chéo Trái (Front-Left)
- **SENSOR_FR** (2): Hướng Chéo Phải (Front-Right)  
- **SENSOR_L** (0): Hướng Trái (Left)
- **SENSOR_R** (3): Hướng Phải (Right)

### Các Hướng Địa Bàn:
```
MAZE_PATH_N = 0 (North/Lên)
MAZE_PATH_E = 1 (East/Phải)
MAZE_PATH_S = 2 (South/Xuống)
MAZE_PATH_W = 3 (West/Trái)
```

---

## 2. Quy Trình Chính (updateFloodPathRun)

### Giai Đoạn 1: MATRIX_DECIDE (Quyết Định)
```
Đầu vào: Robot ở ô hiện tại (matrixX, matrixY), hướng đầu (matrixHeading)

Bước 1: Đọc cảm biến
  - readMapSensorReading() đọc cảm biến FL, FR (và L, R nếu có)
  - Xác định tường ở PHÍA TRƯỚC, TRÁI, PHẢI
  
Bước 2: Cập nhật bản đồ tường
  - updateFloodMapAtNextCell() lưu thông tin tường vào ma trận
  - setMazeWallKnown() đánh dấu tường đã biết
  
Bước 3: Tính Flood Distance
  - computeFloodDistances() dùng BFS từ mục tiêu
  - maze_dist_map[x][y] = khoảng cách ngắn nhất từ mục tiêu
  
Bước 4: Chọn hướng tiếp theo
  - chooseFloodNextDirAt() tính điểm cho mỗi hướng:
    - Khoảng cách đến mục tiêu (quan trọng nhất: dist * 1000)
    - Số lần ghé thăm (visitCost = min(visitCount, 15) * 20)
    - Chi phí rẽ (turnCost: 0 nếu thẳng, 4 nếu rẽ 90°, 10 nếu quay 180°)
    - Chi phí biết (knownCost: 3 nếu tường đã biết)
  
Bước 5: Xác định có cần rẽ không
  - Nếu hướng tiếp theo ≠ hướng hiện tại → queue turn
  - matrixHasQueuedTurn = true, matrixQueuedTurnDir = nextDir
  
Bước 6: Bắt đầu lái xe
  - startMatrixCellDrive() khởi động di chuyển 1 ô
```

### Giai Đoạn 2: MATRIX_TURNING (Rẽ)
```
- updatePointTurn() rẽ xe (nếu cần)
- Sau rẽ xong → MATRIX_DRIVE_CELL
```

### Giai Đoạn 3: MATRIX_DRIVE_CELL (Di Chuyển 1 Ô)
```
- updateOneCellDrive() di chuyển 1 ô về phía trước
- Cập nhật tọa độ: 
  matrixX += dirDx(driveDir)
  matrixY += dirDy(driveDir)
- Lưu lại rằng có thông hành từ ô cũ sang ô mới
- → MATRIX_SETTLE_AT_CELL
```

### Giai Đoạn 4: MATRIX_SETTLE_AT_CELL (Ổn Định)
```
- Dừng xe lại
- Chờ 3 giây (MAZE_CELL_DECISION_WAIT_MS)
- → MATRIX_DECIDE (quay lại bước 1)
```

### Giai Đoạn 5: MATRIX_BACK_CELL (Lùi Lại - Nếu Cần)
```
- Nếu không thể đi tiếp (tất cả hướng đều chặn) → lùi lại
- updateBackDrive() lùi xe 1 ô
- Cập nhật tọa độ ngược lại
- → MATRIX_DECIDE
```

---

## 3. Chi Tiết Hàm Chính

### A. readMapSensorReading()
```cpp
MapSensorReading readMapSensorReading(const IrSnapshot &ir, const RuntimeParams &cfg)
{
  MapSensorReading reading = {};
  
  // Đọc cảm biến bên trái (FL)
  reading.leftWall = readDecisionSideWall(SENSOR_FL, ir, cfg, reading.leftKnown);
  
  // Đọc cảm biến phía trước (L + R)
  reading.frontWall = readDecisionFrontWall(ir, cfg, reading.frontKnown);
  
  // Đọc cảm biến bên phải (FR)
  reading.rightWall = readDecisionSideWall(SENSOR_FR, ir, cfg, reading.rightKnown);
  
  return reading;
}
```

**Điểm Quan Trọng:**
- `leftKnown`, `frontKnown`, `rightKnown` chỉ ra cảm biến có sẵn sàng không
- Nếu không hiệu chỉnh → giá trị không tin cậy
- Nếu biết → `True` = có tường, `False` = không có tường

### B. computeFloodDistances()
```
Dùng BFS (Breadth-First Search) từ mục tiêu (maze_goal_x, maze_goal_y):

1. Khởi tạo: maze_dist_map[goal_x][goal_y] = 0
2. Duyệt BFS từ mục tiêu:
   - Với mỗi ô đã biết khoảng cách
   - Duyệt 4 hướng (N, E, S, W)
   - Nếu không bị chặn bởi tường → đặt khoảng cách = hiện tại + 1
3. Kết quả: maze_dist_map[x][y] = khoảng cách từ (x,y) đến mục tiêu
```

### C. chooseFloodNextDirAt()
```
Tính điểm cho mỗi hướng có thể đi:

score = dist*1000 + visitCost + turnCost + knownCost

Trong đó:
- dist*1000          = khoảng cách đến mục tiêu (ưu tiên cao)
- visitCost          = min(visitCount[nx][ny], 15) * 20  (tránh ghé thăm lặp)
- turnCost           = 0 (thẳng), 4 (90°), 10 (180°)
- knownCost          = 3 (tường đã biết, không khám phá mới)

Chọn hướng có điểm THẤP nhất.
```

---

## 4. Phát Hiện Các Vấn Đề Tiềm Ẩn

### ❌ Vấn Đề 1: Cảm Biến Front Không Chính Xác
**Vị Trí:** `readDecisionFrontWall()`
```cpp
known = leftReady && rightReady;
```
**Vấn Đề:** 
- Chỉ đánh giá front khi CẢ L và R đều sẵn sàng
- Nếu 1 trong 2 không hiệu chỉnh → ignores cả 2
- **Có Thể Thiếu Phát Hiện Tường Ở Phía Trước**

**Khuyến Nghị:**
```cpp
// Nên để: known = leftReady || rightReady;
// hoặc: known = leftReady && rightReady;
// nhưng xử lý từng trường hợp
```

### ❌ Vấn Đề 2: Logic Đọc Tường Bên (Side Wall)
**Vị Trí:** `isMazeWallBySensor()`
```cpp
int diff = wallRef - emptyRef;
if (abs(diff) <= deadband)
  return false;  // ← VẤNĐỀ: Khi diff nhỏ, coi như không có tường
```
**Vấn Đề:**
- Nếu hiệu chỉnh không tốt (max_IR ≈ min_IR) → deadband lớn
- Có thể sai lệch phân biệt tường vs không tường
- **Cảm Biến Có Thể Bị Noise (Nhiễu)**

### ❌ Vấn Đề 3: Queue Turn Khi Cần Lùi
**Vị Trí:** `updateFloodPathRun()` dòng ~1900
```cpp
if (nextCellX == maze_goal_x && nextCellY == maze_goal_y) {
  // ... code xử lý mục tiêu
  startMatrixCellDrive();  // ← Nhưng chưa kiểm tra xem có wall không!
  return false;
}
```
**Vấn Đề:**
- Nếu mục tiêu nằm trong một cul-de-sac (ngõ cụt), code không kiểm tra tường
- **Có Thể Không Xử Lý Được Ngõ Cụt Đúng Cách**

### ❌ Vấn Đề 4: Không Có Mecanisme "Explore Unexplored"
**Vị Trí:** `chooseFloodNextDirAt()` dòng ~850
```cpp
int knownCost = isKnownDirection(x, y, dir) ? 3 : 0;
```
**Vấn Đề:**
- KnownCost quá nhỏ (chỉ 3), so với visitCost có thể lên đến 300
- Robot sẽ ưu tiên tái ghé thăm ô cũ hơn khám phá ô mới
- **Nên Ưu Tiên Khám Phá Ô Chưa Biết**

**Khuyến Nghị:**
```cpp
// Tăng knownCost để ưu tiên khám phá
int knownCost = isKnownDirection(x, y, dir) ? 50 : 0;  // Tăng từ 3→50
```

### ❌ Vấn Đề 5: visitCount Có Thể Overflow
**Vị Trí:** `updateFloodPathRun()` dòng ~1924
```cpp
if (maze_visit_count[matrixX][matrixY] < 255)
  maze_visit_count[matrixX][matrixY]++;
```
**Vấn Đề:**
- `uint8_t` chỉ lên đến 255, nếu ghé thăm quá nhiều lần → bị cứng lại ở 255
- Nhưng đây cũng là cách để tránh overflow
- **Không Phải Lỗi Lớn, Nhưng Nên Chú Ý**

---

## 5. Logic Kiến Nghị Cải Thiện

### A. Cảm Biến Phía Trước Mạnh Hơn
```cpp
bool readDecisionFrontWall(const IrSnapshot &ir, const RuntimeParams &cfg, bool &known)
{
  bool leftReady = isMazeSensorReady(FRONT_STOP_SENSOR_LEFT, cfg);
  bool rightReady = isMazeSensorReady(FRONT_STOP_SENSOR_RIGHT, cfg);
  
  // Thay vì: known = leftReady && rightReady;
  // Nên: known = leftReady || rightReady;  ← Nếu một trong hai sẵn sàng
  
  if (!known)
    return false;
  
  // Nếu cả hai sẵn sàng → và 2 kết quả
  // Nếu một sẵn sàng → lấy kết quả đó
  bool leftWall = leftReady && isMazeFrontWallBySensor(FRONT_STOP_SENSOR_LEFT, ir, cfg);
  bool rightWall = rightReady && isMazeFrontWallBySensor(FRONT_STOP_SENSOR_RIGHT, ir, cfg);
  
  return leftWall || rightWall;  // Nếu một trong hai phát hiện tường
}
```

### B. Tăng Ưu Tiên Khám Phá Ô Mới
```cpp
int knownCost = isKnownDirection(x, y, dir) ? 100 : 0;  // Tăng từ 3→100
```

### C. Xử Lý Tốt Hơn Khi Gặp Ngõ Cụt
```cpp
// Nên thêm logic này trước khi lái xe:
if (!isFloodBlockedAt(matrixX, matrixY, matrixHeading)) {
  // Có thể đi tiếp
  startMatrixCellDrive();
} else {
  // Bị chặn → phải lùi lại
  startMatrixBackCell();
}
```

---

## 6. Công Thức Tính Điểm (Score Formula)

Công thức hiện tại:
```
score = dist*1000 + visitCost + turnCost + knownCost
      = dist*1000 + min(visitCount[nx][ny], 15)*20 + turnCost + knownCost
```

**Ví Dụ:**
- Ô A: dist=10, visitCount=0, turnCost=0, knownCost=0  
  → score = 10*1000 + 0 + 0 + 0 = **10000**

- Ô B: dist=10, visitCount=5, turnCost=4, knownCost=3  
  → score = 10*1000 + 5*20 + 4 + 3 = 10107

- Ô C: dist=11, visitCount=0, turnCost=0, knownCost=0  
  → score = 11*1000 + 0 + 0 + 0 = **11000**

**Chọn:** Ô A (điểm thấp nhất)

---

## 7. Tóm Tắt Chu Trình

```
BẮTĐẦU
  ↓
[MATRIX_DECIDE] - Quyết định
  ├─ Đọc cảm biến (FL, FR, L, R)
  ├─ Cập nhật bản đồ tường
  ├─ Tính Flood Distance
  └─ Chọn hướng tiếp theo
  ↓
[MATRIX_TURNING] - Rẽ (nếu cần)
  ├─ Rẽ xe
  └─ Ổn định hướng
  ↓
[MATRIX_DRIVE_CELL] - Di chuyển
  ├─ Lái xe 1 ô
  └─ Cập nhật tọa độ (x, y)
  ↓
[MATRIX_SETTLE_AT_CELL] - Ổn định
  ├─ Dừng xe
  ├─ Chờ 3 giây
  └─ Quay lại MATRIX_DECIDE
  ↓
[KHI ĐẠTMỤC TIÊU]
  └─ Sinh Path từ Flood Distance
```

