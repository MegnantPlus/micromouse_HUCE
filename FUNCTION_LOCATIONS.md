# 📍 Vị Trí Các Hàm Giải Mê Cung Chính

## 🎯 Hàm Chính

### 1️⃣ updateFloodPathRun() - HÀM CHÍNH
**File:** [src/main.cpp](src/main.cpp)  
**Dòng:** [1835-1989](src/main.cpp#L1835-L1989)  
**Mô Tả:** Hàm chính để giải mê cung bằng Flood Fill

```cpp
bool updateFloodPathRun(const IrSnapshot &rawIr, const IrSnapshot &ir,
                        const RuntimeParams &cfg, int &filteredIrSteer,
                        int boostPwmL, int boostPwmR)
```

**Các Giai Đoạn (matrixPhase):**
- `MATRIX_SETTLE_AT_CELL` (dòng 1847) - Ổn định tại ô
- `MATRIX_TURNING` (dòng 1864) - Rẽ xe
- `MATRIX_DRIVE_CELL` (dòng 1883) - Di chuyển 1 ô
- `MATRIX_DECIDE` (dòng 1913) - Quyết định hướng

---

### 2️⃣ chooseFloodNextDirAt() - CHỌN HƯỚNG
**File:** [src/main.cpp](src/main.cpp)  
**Dòng:** [815-850](src/main.cpp#L815-L850)  
**Mô Tả:** Tính điểm cho mỗi hướng, chọn hướng có điểm thấp nhất

```cpp
uint8_t chooseFloodNextDirAt(int x, int y, uint8_t heading,
                             const RuntimeParams &cfg)
```

**Công Thức Tính Điểm:**
```
score = dist*1000 + visitCost + turnCost + knownCost
```

---

### 3️⃣ computeFloodDistances() - TÍNH KHOẢNG CÁCH
**File:** [src/main.cpp](src/main.cpp)  
**Dòng:** [763-814](src/main.cpp#L763-L814)  
**Mô Tả:** Dùng BFS từ mục tiêu tính khoảng cách đến mỗi ô

```cpp
void computeFloodDistances()
```

---

## 📋 Các Hàm Hỗ Trợ Quan Trọng

### 4️⃣ readMapSensorReading() - ĐỌC CẢM BIẾN
**File:** [src/main.cpp](src/main.cpp)  
**Dòng:** [588-600](src/main.cpp#L588-L600)  
**Mô Tả:** Đọc cảm biến FL, FR, L, R để phát hiện tường

```cpp
MapSensorReading readMapSensorReading(const IrSnapshot &ir,
                                      const RuntimeParams &cfg)
```

---

### 5️⃣ updateFloodMapAtNextCell() - CẬP NHẬT BẢN ĐỒ TƯỜNG
**File:** [src/main.cpp](src/main.cpp)  
**Dòng:** [639-674](src/main.cpp#L639-L674)  
**Mô Tả:** Cập nhật ma trận tường dựa trên cảm biến đọc được

```cpp
MapSensorReading updateFloodMapAtNextCell(const IrSnapshot &ir,
                                          const RuntimeParams &cfg,
                                          int &nextCellX, int &nextCellY)
```

---

### 6️⃣ readDecisionFrontWall() - ĐỌC TƯỜNG PHÍA TRƯỚC
**File:** [src/main.cpp](src/main.cpp)  
**Dòng:** [596-604](src/main.cpp#L596-L604)  
**Mô Tả:** **⚠️ VẤNĐỀ:** Chỉ đọc khi CẢ L và R sẵn sàng

```cpp
bool readDecisionFrontWall(const IrSnapshot &ir, const RuntimeParams &cfg,
                           bool &known)
```

**VỊ TRÍ VẤN ĐỀ 1:**
```cpp
known = leftReady && rightReady;  // ← WRONG: Phải là || không phải &&
```

---

### 7️⃣ isFloodBlockedAt() - KIỂM TRA TỦỜ CHẶN
**File:** [src/main.cpp](src/main.cpp)  
**Dòng:** [302-314](src/main.cpp#L302-L314)  
**Mô Tả:** Kiểm tra xem hướng nào bị tường chặn

```cpp
bool isFloodBlockedAt(int x, int y, uint8_t dir)
```

---

## 🔴 ĐỊA ĐIỂM VẤN ĐỀ

### Problem 1: Front Sensor Logic ❌
**Vị Trí:** [src/main.cpp:601](src/main.cpp#L601)
```cpp
known = leftReady && rightReady;  // ← Phải là ||
```
**Sửa thành:**
```cpp
known = leftReady || rightReady;
```

---

### Problem 2: knownCost Quá Nhỏ ❌
**Vị Trí:** [src/main.cpp:834](src/main.cpp#L834)
```cpp
int knownCost = isKnownDirection(x, y, dir) ? 3 : 0;  // ← Quá nhỏ
```
**Sửa thành:**
```cpp
int knownCost = isKnownDirection(x, y, dir) ? 100 : 0;
```

---

### Problem 3: Không Validate Tường Trước Drive ❌
**Vị Trí:** [src/main.cpp:1980-1985](src/main.cpp#L1980-L1985)
```cpp
matrixNextDir = matrixHeading;
debugMazeNextDir =
    matrixHasQueuedTurn ? matrixQueuedTurnDir : matrixNextDir;
startMatrixCellDrive();  // ← Chưa check isFloodBlockedAt()
```
**Nên thêm:**
```cpp
if (!isFloodBlockedAt(matrixX, matrixY, matrixHeading)) {
  startMatrixCellDrive();
} else {
  // Error handling
  return true;
}
```

---

## 🧪 Debug Points Để Thêm Log

### 1. Trong updateFloodPathRun() dòng ~1930
```cpp
// Thêm để xem robot quyết định hướng nào
Serial.printf("[MAZE] Pos:(%d,%d) Head:%d Next:%d Dist:%d\n",
              matrixX, matrixY, matrixHeading, matrixNextDir,
              maze_dist_map[matrixX][matrixY]);
```

### 2. Trong chooseFloodNextDirAt() dòng ~840
```cpp
// Xem chi tiết scoring
Serial.printf("[SCORE] %d:%d | dist:%d visit:%d turn:%d known:%d = %d\n",
              x, y, direction, dist, visit_cost, turn_cost, known_cost, score);
```

### 3. Trong readMapSensorReading() dòng ~595
```cpp
// Xem cảm biến đọc được gì
Serial.printf("[SENSE] F:%s L:%s R:%s\n",
              reading.frontWall?"W":"O",
              reading.leftWall?"W":"O",
              reading.rightWall?"W":"O");
```

### 4. Trong isFloodBlockedAt()
```cpp
// Cảnh báo nếu bị tường chặn
if (isKnownWall(x, y, dir)) {
  Serial.printf("[BLOCKED] Wall at (%d,%d) dir:%d\n", x, y, dir);
}
```

---

## 📊 Gọi Chuỗi (Call Stack)

```
updateFloodPathRun()  [1835]
    │
    ├─→ updateFloodMapAtNextCell()  [1928]
    │       └─→ readMapSensorReading()  [643]
    │           └─→ readDecisionFrontWall()  [596] ⚠️ PROBLEM
    │           └─→ readDecisionSideWall()  [567]
    │
    ├─→ computeFloodDistances()  [1929]
    │       └─→ BFS từ goal_x, goal_y
    │
    └─→ chooseFloodNextDirAt()  [1951] ⚠️ PROBLEM (line 834)
            └─→ Tính score cho mỗi hướng
            └─→ Return hướng có score thấp nhất
```

---

## 🎯 Để Debug, Mở File Này

1. **Main Loop:** File [src/main.cpp](src/main.cpp), dòng **1835** (`updateFloodPathRun`)
2. **Direction Choice:** File [src/main.cpp](src/main.cpp), dòng **815** (`chooseFloodNextDirAt`)
3. **Distance Calculation:** File [src/main.cpp](src/main.cpp), dòng **763** (`computeFloodDistances`)
4. **Sensor Reading:** File [src/main.cpp](src/main.cpp), dòng **588** (`readMapSensorReading`)

---

## ⚡ Quick Jump

Nhấn Ctrl+G rồi nhập dòng số để nhảy:
- `1835` - Main flood fill loop
- `815` - Direction scoring
- `763` - Flood distance calculation
- `601` - Front sensor (BUG)
- `834` - knownCost (BUG)
- `1980` - Missing wall check (BUG)

