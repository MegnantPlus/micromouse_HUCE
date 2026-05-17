# Sơ Đồ & Hình Ảnh Minh Họa

## 🔄 State Machine Diagram

```
                    ┌─────────────────────────────────────┐
                    │      START / RESET                  │
                    │  resetFloodRunner()                 │
                    └──────────────┬──────────────────────┘
                                   │
                                   ▼
                    ┌─────────────────────────────────────┐
                    │  MATRIX_DECIDE                      │
                    │  • Đọc cảm biến (FL, FR, L, R)      │
                    │  • Cập nhật bản đồ tường            │
                    │  • Tính Flood Distance (BFS)        │
                    │  • Chọn hướng tiếp theo             │
                    │  • Queue turn nếu cần               │
                    └──────────────┬──────────────────────┘
                                   │
                    ┌──────────────┴──────────────────┐
                    │                                 │
                    ▼                                 ▼
         ┌──────────────────────┐        ┌──────────────────────┐
         │  MATRIX_TURNING      │        │ MATRIX_DRIVE_CELL    │
         │  (nếu cần rẽ)        │        │ (nếu thẳng)          │
         │  • updatePointTurn() │        │ • Drive 1 cell       │
         │  • Rẽ xe             │        │ • Update coord       │
         │  • Ổn định hướng     │        │ • Mark passage       │
         │  → MATRIX_DRIVE_CELL │        └──────────┬───────────┘
         └──────────────────────┘                    │
                                                     ▼
                                        ┌──────────────────────┐
                                        │MATRIX_SETTLE_AT_CELL │
                                        │ • Brake motors       │
                                        │ • Wait 3 seconds     │
                                        │ • → MATRIX_DECIDE    │
                                        └─────────┬────────────┘
                                                  │
                                    ┌─────────────┴──────────────┐
                                    │                            │
                                    ▼                            ▼
                         ┌─────────────────────┐   ┌────────────────────┐
                         │  GOAL REACHED?      │   │  Continue Explore  │
                         │  maze_goal_x/y ==   │   │  → MATRIX_DECIDE   │
                         │  matrixX/Y          │   └────────────────────┘
                         │  YES → SUCCESS      │
                         └─────────────────────┘
                                    │
                                    ▼
                         ┌─────────────────────┐
                         │ generatePathFromFF()│
                         │ Generate optimal    │
                         │ path to goal        │
                         │ → COMPLETE          │
                         └─────────────────────┘
```

---

## 📐 Scoring Formula Visualization

```
                    Choosing Next Direction
                    
    From Cell (x,y) = 4 choices: N, E, S, W
            │
            ▼
    ┌─────────────────────────────────────────┐
    │ For each direction:                      │
    │ • Check if blocked                       │
    │ • Get neighbor cell distance             │
    │                                          │
    │ Score = d*1000 + visit*20 + turn + known│
    └─────────────────────────────────────────┘
            │
            ├─ dist*1000      (most important)
            │  ▼
            │  5: score += 5000  ← choose this
            │  6: score += 6000
            │
            ├─ visitCost*20   (avoid revisit)
            │  ▼
            │  visit 0: +0      ← new cell
            │  visit 5: +100    ← revisited
            │  visit 15+: +300  ← heavily visited
            │
            ├─ turnCost       (prefer straight)
            │  ▼
            │  0°:   +0        ← straight
            │  90°:  +4        ← turn
            │  180°: +10       ← U-turn
            │
            └─ knownCost      (explore unknown)
               ▼
               known: +100     ← already mapped
               unknown: +0     ← new wall info
```

---

## 🎯 Sensor Reading Logic

```
Robot Position = (1,1)
Heading Direction = N (North/Lên)

    Sensors:
    ┌────────┐
    │   FL   │  ← Diagonal Left (hướng W)
    │  /  \  │
    │ L  R  │  ← Straight Left/Right (W & E)
    │  \  /  │
    │   FR   │  ← Diagonal Right (hướng E)
    └────────┘

Reading:
    Front (N):   readDecisionFrontWall(L, R)
                 ├─ SENSOR_L (straight front-left)
                 └─ SENSOR_R (straight front-right)
    
    Left (W):    readDecisionSideWall(FL)
                 └─ SENSOR_FL (diagonal left)
    
    Right (E):   readDecisionSideWall(FR)
                 └─ SENSOR_FR (diagonal right)

Wall Map Update:
    ┌────────────────────────────────────┐
    │ Current cell (1,1):                │
    │   setMazeWallKnown(1, 1, N, val)   │
    │   setMazeWallKnown(1, 1, W, val)   │
    │   setMazeWallKnown(1, 1, E, val)   │
    │                                    │
    │ Also update neighbors:             │
    │   setMazeWallKnown(1, 2, S, ...)   │
    │   setMazeWallKnown(0, 1, E, ...)   │
    │   setMazeWallKnown(2, 1, W, ...)   │
    └────────────────────────────────────┘
```

---

## 🔁 Flood Fill BFS Process

```
Goal at (7,7)

Step 1: Initialize
   maze_dist_map[7][7] = 0
   queue = [(7,7)]

Step 2: BFS Expansion
   
   Distance 0:        Distance 1:        Distance 2:
   ┌─────────┐       ┌─────────┐       ┌─────────┐
   │ 0 1 2 3 │       │ 0 1 2 3 │       │ 0 1 2 3 │
   │ 1   G   │  -->  │ 1 1 G 1 │  -->  │ 1 1 G 1 │
   │ 2 3 2 1 │       │ 2 3 2 1 │       │ 2 3 2 1 │
   │ 3 2 1 0 │       │ 3 2 1 0 │       │ 3 2 1 0 │
   └─────────┘       └─────────┘       └─────────┘
   
   (G = Goal = 0)
   (Higher number = farther from goal)

Step 3: Robot Decision
   From (0,0), distances to neighbors:
   ├─ N (up):    dist[0][1] = 3
   ├─ E (right): dist[1][0] = 2  ← CHOOSE (lowest)
   ├─ S (down):  BLOCKED
   └─ W (left):  BLOCKED
```

---

## ❌ Issue 1: Front Sensor Problem

```
Current Logic (WRONG):
    bool leftReady = isMazeSensorReady(SENSOR_L);
    bool rightReady = isMazeSensorReady(SENSOR_R);
    known = leftReady && rightReady;  // ← WRONG: Both must be ready
    
    Scenario:
    ┌─────────────────────────────────┐
    │ SENSOR_L: Calibrated ✓          │
    │ SENSOR_R: Not calibrated ✗      │
    │                                 │
    │ known = true && false = FALSE   │
    │ → Robot ignores front wall!     │ ← Problem!
    └─────────────────────────────────┘

Fixed Logic (CORRECT):
    bool leftReady = isMazeSensorReady(SENSOR_L);
    bool rightReady = isMazeSensorReady(SENSOR_R);
    known = leftReady || rightReady;  // ← CORRECT: At least one
    
    if (leftReady && rightReady) {
      return leftWall || rightWall;  // Both available
    } else if (leftReady) {
      return leftWall;               // Only left
    } else {
      return rightWall;              // Only right
    }
    
    Result:
    ┌─────────────────────────────────┐
    │ known = true || false = TRUE    │
    │ Use SENSOR_L to detect wall    │
    │ → Correct front wall detection  │ ← Fixed!
    └─────────────────────────────────┘
```

---

## 🎲 Issue 2: knownCost Problem

```
Comparison of Direction Choices:

Scenario: 3 options from cell (5,5)

┌─────────────────────────────────────────────────────────────┐
│ Option A: New Cell (never visited)                          │
│   dist = 5, visitCount = 0, turnCost = 0, knownCost = 0   │
│   score = 5*1000 + 0 + 0 + 0 = 5000  ← Good              │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Option B: Old Cell (visited 5 times) - CURRENT CODE         │
│   dist = 5, visitCount = 5, turnCost = 0, knownCost = 3   │
│   score = 5*1000 + 100 + 0 + 3 = 5103  ← BAD             │
│   → Choice: Option A (correct, but by luck!)               │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Option C: Heavier revisited (visited 10 times)              │
│   dist = 5, visitCount = 10, turnCost = 0, knownCost = 3  │
│   score = 5*1000 + 200 + 0 + 3 = 5203  ← STILL BAD       │
│   → Choice: Option B (WRONG! Should choose A)              │
└─────────────────────────────────────────────────────────────┘

AFTER FIX (knownCost = 100):

┌─────────────────────────────────────────────────────────────┐
│ Option A: New Cell (never visited)                          │
│   dist = 5, visitCount = 0, turnCost = 0, knownCost = 0   │
│   score = 5*1000 + 0 + 0 + 0 = 5000  ← STILL Best        │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Option B: Old Cell (visited 5 times) - FIXED CODE           │
│   dist = 5, visitCount = 5, turnCost = 0, knownCost = 100 │
│   score = 5*1000 + 100 + 0 + 100 = 5200  ← WORSE         │
│   → Choice: Option A (CORRECT!)                            │
└─────────────────────────────────────────────────────────────┘
```

---

## 📊 Movement Tracking

```
Example Path Execution:

┌─────────────────────────────────────┐
│ Step 1: At (0,0), Heading = N       │
│                                     │
│   ┌─┬─┬─┐                           │
│   │R│ │ │ R = Robot, facing N      │
│   ├─┼─┼─┤                           │
│   │ │ │ │                           │
│   ├─┼─┼─┤                           │
│   │ │ │G│ G = Goal                 │
│   └─┴─┴─┘                           │
│                                     │
│ Sensors: F=open, L=wall, R=open    │
│ Decision: Go North                  │
│ Action: Move(0,1)                  │
└─────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────┐
│ Step 2: At (0,1), Heading = N       │
│                                     │
│   ┌─┬─┬─┐                           │
│   │ │R│ │ Robot moved               │
│   ├─┼─┼─┤                           │
│   │ │ │ │                           │
│   ├─┼─┼─┤                           │
│   │ │ │G│                           │
│   └─┴─┴─┘                           │
│                                     │
│ Sensors: F=open, L=open, R=wall    │
│ Decision: Go left (W)               │
│ Action: Turn left, Move(-1,1)      │
└─────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────┐
│ Step N: At (2,2), Heading = ?       │
│                                     │
│   ┌─┬─┬─┐                           │
│   │ │ │ │                           │
│   ├─┼─┼─┤                           │
│   │ │ │R│ Robot reached goal!       │
│   ├─┼─┼─┤                           │
│   │ │ │G│                           │
│   └─┴─┴─┘                           │
│                                     │
│ Action: STOP - Goal reached!        │
└─────────────────────────────────────┘
```

---

## 🔧 Fix Priority Pyramid

```
                    ┌──────────────────┐
                    │  Fix All 5       │
                    │  "Perfect"       │
                    └────────┬─────────┘
                             △
                            ╱ ╲
                           ╱   ╲
                    ┌─────────────────┐
                    │  Fix 1,2,3      │
                    │  "Very Good"    │
                    │ 90% of benefit  │
                    └────────┬────────┘
                             △
                            ╱ ╲
                           ╱   ╲
                    ┌─────────────────┐
                    │  Fix 1,2        │
                    │  "Good"         │
                    │ 70% of benefit  │
                    └────────┬────────┘
                             △
                            ╱ ╲
                           ╱   ╲
                    ┌─────────────────┐
                    │  Fix 1 Only     │
                    │  "Basic"        │
                    │ 50% of benefit  │
                    └────────┬────────┘
                             △
                            ╱ ╲
                           ╱   ╲
                    ┌─────────────────┐
                    │  No Fix         │
                    │  "Current"      │
                    │ Crashes happen  │
                    └─────────────────┘
```

Recommended: **Fix 1, 2, 3** for best ROI

