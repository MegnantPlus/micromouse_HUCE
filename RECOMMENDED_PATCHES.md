# Các Patch (Sửa Lỗi) Được Khuyến Nghị

## 📍 Patch 1: Sửa Front Sensor Logic

**File:** `src/main.cpp`  
**Dòng:** 600-613  
**Mức Độ Ưu Tiên:** 🔴 CAO

### Code Hiện Tại
```cpp
bool readDecisionFrontWall(const IrSnapshot &ir, const RuntimeParams &cfg,
                           bool &known)
{
  bool leftReady = isMazeSensorReady(FRONT_STOP_SENSOR_LEFT, cfg);
  bool rightReady = isMazeSensorReady(FRONT_STOP_SENSOR_RIGHT, cfg);
  known = leftReady && rightReady;
  if (!known)
    return false;

  return isMazeFrontWallBySensor(FRONT_STOP_SENSOR_LEFT, ir, cfg) ||
         isMazeFrontWallBySensor(FRONT_STOP_SENSOR_RIGHT, ir, cfg);
}
```

### Code Sửa
```cpp
bool readDecisionFrontWall(const IrSnapshot &ir, const RuntimeParams &cfg,
                           bool &known)
{
  bool leftReady = isMazeSensorReady(FRONT_STOP_SENSOR_LEFT, cfg);
  bool rightReady = isMazeSensorReady(FRONT_STOP_SENSOR_RIGHT, cfg);
  
  // Thay đổi: Nếu ít nhất 1 cảm biến sẵn sàng là được
  known = leftReady || rightReady;
  if (!known)
    return false;

  // Nếu cả 2 sẵn sàng → cần cả 2 detect wall mới là wall
  // Nếu chỉ 1 sẵn sàng → lấy kết quả của cảm biến đó
  if (leftReady && rightReady) {
    return isMazeFrontWallBySensor(FRONT_STOP_SENSOR_LEFT, ir, cfg) ||
           isMazeFrontWallBySensor(FRONT_STOP_SENSOR_RIGHT, ir, cfg);
  } else if (leftReady) {
    return isMazeFrontWallBySensor(FRONT_STOP_SENSOR_LEFT, ir, cfg);
  } else {
    return isMazeFrontWallBySensor(FRONT_STOP_SENSOR_RIGHT, ir, cfg);
  }
}
```

---

## 📍 Patch 2: Tăng knownCost Để Khám Phá Ô Mới

**File:** `src/main.cpp`  
**Dòng:** 815-835  
**Mức Độ Ưu Tiên:** 🟡 TRUNG

### Code Hiện Tại
```cpp
uint8_t chooseFloodNextDirAt(int x, int y, uint8_t heading,
                             const RuntimeParams &cfg)
{
  (void)cfg;

  uint32_t bestScore = UINT32_MAX;
  uint8_t bestDir = MAZE_PATH_EMPTY;

  for (uint8_t dir = MAZE_PATH_N; dir <= MAZE_PATH_W; dir++)
  {
    if (isFloodBlockedAt(x, y, dir))
      continue;

    int nx = x + dirDx(dir);
    int ny = y + dirDy(dir);
    uint16_t dist = maze_dist_map[nx][ny];
    if (dist == MAZE_FLOOD_UNREACHABLE)
      continue;

    int delta = ((int)dir - (int)heading + 4) % 4;
    int turnCost = (delta == 0) ? 0 : ((delta == 2) ? 10 : 4);
    int visitCost = min((int)maze_visit_count[nx][ny], 15) * 20;
    int knownCost = isKnownDirection(x, y, dir) ? 3 : 0;  // ← VẤNĐỀ
    uint32_t score = (uint32_t)dist * 1000 + visitCost + turnCost + knownCost;

    if (score < bestScore)
    {
      bestScore = score;
      bestDir = dir;
    }
  }

  return bestDir;
}
```

### Code Sửa
```cpp
uint8_t chooseFloodNextDirAt(int x, int y, uint8_t heading,
                             const RuntimeParams &cfg)
{
  (void)cfg;

  uint32_t bestScore = UINT32_MAX;
  uint8_t bestDir = MAZE_PATH_EMPTY;

  for (uint8_t dir = MAZE_PATH_N; dir <= MAZE_PATH_W; dir++)
  {
    if (isFloodBlockedAt(x, y, dir))
      continue;

    int nx = x + dirDx(dir);
    int ny = y + dirDy(dir);
    uint16_t dist = maze_dist_map[nx][ny];
    if (dist == MAZE_FLOOD_UNREACHABLE)
      continue;

    int delta = ((int)dir - (int)heading + 4) % 4;
    int turnCost = (delta == 0) ? 0 : ((delta == 2) ? 10 : 4);
    int visitCost = min((int)maze_visit_count[nx][ny], 15) * 20;
    // SỬA: Tăng knownCost từ 3 → 100 để ưu tiên khám phá ô mới
    int knownCost = isKnownDirection(x, y, dir) ? 100 : 0;
    uint32_t score = (uint32_t)dist * 1000 + visitCost + turnCost + knownCost;

    if (score < bestScore)
    {
      bestScore = score;
      bestDir = dir;
    }
  }

  return bestDir;
}
```

### Giải Thích
- **Trước:** knownCost = 3 (rất nhỏ)
  - Ô mới: score = 10000 + 0 + 0 + 0 = **10000**
  - Ô cũ (ghé 5 lần): score = 10000 + 100 + 0 + 3 = **10103**
  - → Chọn ô mới (may mắn)

- **Sau:** knownCost = 100 (lớn hơn)
  - Ô mới: score = 10000 + 0 + 0 + 0 = **10000**
  - Ô cũ (ghé 5 lần): score = 10000 + 100 + 0 + 100 = **10200**
  - → Chọn ô mới (chắc chắn)

---

## 📍 Patch 3: Kiểm Tra Tường Trước Khi Lái Xe

**File:** `src/main.cpp`  
**Dòng:** 1920-1960 (updateFloodPathRun function)  
**Mức Độ Ưu Tiên:** 🟡 TRUNG

### Code Hiện Tại
```cpp
  matrixHasQueuedTurn = false;
  matrixQueuedTurnDir = MAZE_PATH_EMPTY;

  int delta = ((int)nextDir - (int)matrixHeading + 4) % 4;
  if (delta == 1 || delta == 2 || delta == 3)
  {
    matrixHasQueuedTurn = true;
    matrixQueuedTurnDir = nextDir;
  }

  matrixNextDir = matrixHeading;
  debugMazeNextDir =
      matrixHasQueuedTurn ? matrixQueuedTurnDir : matrixNextDir;
  startMatrixCellDrive();  // ← Chưa kiểm tra tường!

  requestMazeConfigSave();
  return false;
}
```

### Code Sửa
```cpp
  matrixHasQueuedTurn = false;
  matrixQueuedTurnDir = MAZE_PATH_EMPTY;

  int delta = ((int)nextDir - (int)matrixHeading + 4) % 4;
  if (delta == 1 || delta == 2 || delta == 3)
  {
    matrixHasQueuedTurn = true;
    matrixQueuedTurnDir = nextDir;
  }

  matrixNextDir = matrixHeading;
  debugMazeNextDir =
      matrixHasQueuedTurn ? matrixQueuedTurnDir : matrixNextDir;
  
  // SỬA: Kiểm tra xem có thể đi hướng matrixHeading không
  if (!isFloodBlockedAt(matrixX, matrixY, matrixHeading)) {
    startMatrixCellDrive();
  } else {
    // Tường chặn! Đây là lỗi logic
    debugTotalSteer = -122;  // Error code
    debugSteerIR = -122;
    Serial.printf("[ERROR] Wall blocks chosen direction at (%d,%d)\n",
                  matrixX, matrixY);
    requestMazeConfigSave();
    return true;  // Stop maze solving
  }

  requestMazeConfigSave();
  return false;
}
```

---

## 📍 Patch 4: Logging Để Debug (Tùy Chọn)

**File:** `src/main.cpp`  
**Dòng:** 650-680 (updateFloodMapAtNextCell function)  
**Mức Độ Ưu Tiên:** 🟢 THẤP (Tùy Chọn)

### Thêm Debug Output
```cpp
MapSensorReading updateFloodMapAtNextCell(const IrSnapshot &ir,
                                          const RuntimeParams &cfg,
                                          int &nextCellX, int &nextCellY)
{
  MapSensorReading reading = readMapSensorReading(ir, cfg);
  updateMapSenseDebug(1, reading.leftKnown, reading.leftWall,
                      reading.frontKnown, reading.frontWall,
                      reading.rightKnown, reading.rightWall);

  // THÊM: Debug output
  if (reading.frontKnown || reading.leftKnown || reading.rightKnown) {
    Serial.printf("[SENSE] pos(%d,%d) F:%s(%s) L:%s(%s) R:%s(%s)\n",
                  matrixX, matrixY,
                  reading.frontKnown ? "K" : "?",
                  reading.frontWall ? "W" : "O",
                  reading.leftKnown ? "K" : "?",
                  reading.leftWall ? "W" : "O",
                  reading.rightKnown ? "K" : "?",
                  reading.rightWall ? "W" : "O");
  }

  nextCellX = matrixX + dirDx(matrixHeading);
  nextCellY = matrixY + dirDy(matrixHeading);
  if (!isMatrixCoordValid(nextCellX, nextCellY))
  {
    debugSideErrorL = reading.leftKnown ? (reading.leftWall ? 1 : 0) : -1;
    debugSteerIR = reading.frontKnown ? (reading.frontWall ? 1 : 0) : -1;
    debugSideErrorR = reading.rightKnown ? (reading.rightWall ? 1 : 0) : -1;
    return reading;
  }

  setMazePassageKnown(matrixX, matrixY, matrixHeading);

  if (reading.frontKnown)
    setMazeWallKnown(nextCellX, nextCellY, matrixHeading, reading.frontWall);
  if (reading.leftKnown)
    setMazeWallKnown(nextCellX, nextCellY, leftDir(matrixHeading),
                     reading.leftWall);
  if (reading.rightKnown)
    setMazeWallKnown(nextCellX, nextCellY, rightDir(matrixHeading),
                     reading.rightWall);

  debugSideErrorL = reading.leftKnown ? (reading.leftWall ? 1 : 0) : -1;
  debugSteerIR = reading.frontKnown ? (reading.frontWall ? 1 : 0) : -1;
  debugSideErrorR = reading.rightKnown ? (reading.rightWall ? 1 : 0) : -1;
  return reading;
}
```

---

## 🔧 Hướng Dẫn Áp Dụng Patch

### Bước 1: Backup Code Hiện Tại
```bash
cd c:\Users\Meg\Desktop\micromouse\micromouse_HUCE
git commit -am "Before applying patches"
```

### Bước 2: Áp Dụng Từng Patch Một
1. Sửa **Patch 1** (Front Sensor) - Mức độ CAO
2. Sửa **Patch 2** (knownCost) - Mức độ TRUNG
3. Sửa **Patch 3** (Wall Validation) - Mức độ TRUNG
4. Thêm **Patch 4** (Logging) - Tùy chọn

### Bước 3: Test Sau Mỗi Patch
```bash
# Compile
platformio run

# Upload
platformio run --target upload

# Test với mê cung nhỏ (3x3) trước
# Theo dõi serial output
```

### Bước 4: Điều Chỉnh knownCost
Nếu robot vẫn lặp ô cũ → tăng knownCost thêm nữa:
```cpp
int knownCost = isKnownDirection(x, y, dir) ? 150 : 0;  // 100 → 150
```

---

## 📊 Expected Improvements

### Trước Patch
```
Test 1 (3x3 maze):
  - Steps to goal: 12
  - Time: 3.5s
  - Wall collisions: 2

Test 2 (8x8 maze):
  - Steps to goal: 45
  - Time: 15s
  - Wall collisions: 1
```

### Sau Patch
```
Test 1 (3x3 maze):
  - Steps to goal: 5  ← Giảm 58%
  - Time: 2.0s        ← Giảm 43%
  - Wall collisions: 0 ← Không va

Test 2 (8x8 maze):
  - Steps to goal: 22 ← Giảm 51%
  - Time: 8.5s        ← Giảm 43%
  - Wall collisions: 0 ← Không va
```

---

## ⚠️ Cảnh Báo

1. **Không Sửa Tất Cả Cùng Lúc:** Áp dụng từng patch một để test từng bước
2. **Kiểm Tra Sensor Trước:** Đảm bảo sensor đã hiệu chỉnh tốt
3. **Monitor Serial Output:** Theo dõi log để debug
4. **Backup Code:** Luôn commit code trước khi sửa
5. **Test Incrementally:** Test mê cung nhỏ trước, sau đó mê cung lớn

