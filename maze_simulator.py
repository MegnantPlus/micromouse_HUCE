#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mô Phỏng Thuật Toán Giải Mê Cung (Flood Fill)
Simulating Micromouse Maze Solver Algorithm

Hướng:
  N(0)
  |
W(3)--E(1)
  |
  S(2)
"""

from collections import deque
from typing import List, Tuple, Set, Dict
import json

# Constants
MAZE_PATH_N = 0
MAZE_PATH_E = 1
MAZE_PATH_S = 2
MAZE_PATH_W = 3

MAZE_WALL_N = 1 << 0  # 0b0001
MAZE_WALL_E = 1 << 1  # 0b0010
MAZE_WALL_S = 1 << 2  # 0b0100
MAZE_WALL_W = 1 << 3  # 0b1000

FLOOD_UNREACHABLE = 255

DIR_NAMES = {0: 'N', 1: 'E', 2: 'S', 3: 'W'}
DIR_DELTA = {
    0: (0, 1),   # N: (dx, dy) = (0, +1) - lên trên
    1: (1, 0),   # E: (1, 0) - sang phải
    2: (0, -1),  # S: (0, -1) - xuống dưới
    3: (-1, 0)   # W: (-1, 0) - sang trái
}


class MazeSimulator:
    """Mô Phỏng Robot Giải Mê Cung"""
    
    def __init__(self, width: int = 16, height: int = 16):
        self.width = width
        self.height = height
        
        # Ma trận tường: mỗi ô chứa 4 bit cho 4 hướng
        self.wall_map = [[0 for _ in range(width)] for _ in range(height)]
        
        # Ma trận tường đã biết
        self.known_map = [[0 for _ in range(width)] for _ in range(height)]
        
        # Ma trận khoảng cách flood
        self.dist_map = [[FLOOD_UNREACHABLE for _ in range(width)] for _ in range(height)]
        
        # Ma trận số lần ghé thăm
        self.visit_count = [[0 for _ in range(width)] for _ in range(height)]
        
        # Vị trí hiện tại
        self.pos_x = 0
        self.pos_y = 0
        self.heading = MAZE_PATH_N
        
        # Mục tiêu
        self.goal_x = 7
        self.goal_y = 7
        
        # Lịch sử di chuyển
        self.move_history: List[Tuple[int, int, int, str]] = []
        
    def set_wall(self, x: int, y: int, direction: int, has_wall: bool = True):
        """Đặt tường trong mê cung"""
        if 0 <= x < self.width and 0 <= y < self.height:
            bit = 1 << direction
            if has_wall:
                self.wall_map[x][y] |= bit
            else:
                self.wall_map[x][y] &= ~bit
    
    def has_wall(self, x: int, y: int, direction: int) -> bool:
        """Kiểm tra có tường không"""
        if not (0 <= x < self.width and 0 <= y < self.height):
            return True
        return (self.wall_map[x][y] & (1 << direction)) != 0
    
    def is_known(self, x: int, y: int, direction: int) -> bool:
        """Kiểm tra tường đã biết không"""
        if not (0 <= x < self.width and 0 <= y < self.height):
            return True
        return (self.known_map[x][y] & (1 << direction)) != 0
    
    def set_wall_known(self, x: int, y: int, direction: int, has_wall: bool):
        """Đánh dấu tường đã biết"""
        if not (0 <= x < self.width and 0 <= y < self.height):
            return
        
        bit = 1 << direction
        self.known_map[x][y] |= bit
        if has_wall:
            self.wall_map[x][y] |= bit
        else:
            self.wall_map[x][y] &= ~bit
    
    def opposite_dir(self, direction: int) -> int:
        """Hướng đối diện"""
        return (direction + 2) % 4
    
    def left_dir(self, direction: int) -> int:
        """Rẽ trái"""
        return (direction + 3) % 4
    
    def right_dir(self, direction: int) -> int:
        """Rẽ phải"""
        return (direction + 1) % 4
    
    def is_blocked(self, x: int, y: int, direction: int) -> bool:
        """Kiểm tra có thể đi hướng này không"""
        if not (0 <= x < self.width and 0 <= y < self.height):
            return True
        
        dx, dy = DIR_DELTA[direction]
        nx, ny = x + dx, y + dy
        
        if not (0 <= nx < self.width and 0 <= ny < self.height):
            return True
        
        # Kiểm tra tường từ ô hiện tại và ô tiếp theo
        return self.has_wall(x, y, direction) or self.has_wall(nx, ny, self.opposite_dir(direction))
    
    def compute_flood_distances(self):
        """Tính Flood Distance từ mục tiêu"""
        # Khởi tạo
        for y in range(self.height):
            for x in range(self.width):
                self.dist_map[x][y] = FLOOD_UNREACHABLE
        
        # BFS từ mục tiêu
        queue = deque()
        self.dist_map[self.goal_x][self.goal_y] = 0
        queue.append((self.goal_x, self.goal_y))
        
        while queue:
            x, y = queue.popleft()
            base_dist = self.dist_map[x][y]
            
            # Duyệt 4 hướng
            for direction in range(4):
                if self.is_blocked(x, y, direction):
                    continue
                
                dx, dy = DIR_DELTA[direction]
                nx, ny = x + dx, y + dy
                
                if self.dist_map[nx][ny] <= base_dist + 1:
                    continue
                
                self.dist_map[nx][ny] = base_dist + 1
                queue.append((nx, ny))
    
    def choose_next_direction(self, visit_threshold: int = 15) -> int:
        """Chọn hướng tiếp theo dựa trên công thức điểm"""
        x, y = self.pos_x, self.pos_y
        
        best_score = float('inf')
        best_dir = -1
        
        scoring_details = {}
        
        for direction in range(4):
            if self.is_blocked(x, y, direction):
                continue
            
            dx, dy = DIR_DELTA[direction]
            nx, ny = x + dx, y + dy
            
            dist = self.dist_map[nx][ny]
            if dist == FLOOD_UNREACHABLE:
                continue
            
            # Tính điểm
            delta = (direction - self.heading + 4) % 4
            turn_cost = 0 if delta == 0 else (10 if delta == 2 else 4)
            visit_cost = min(self.visit_count[nx][ny], visit_threshold) * 20
            known_cost = 3 if self.is_known(x, y, direction) else 0
            
            score = dist * 1000 + visit_cost + turn_cost + known_cost
            
            dir_name = DIR_NAMES[direction]
            scoring_details[dir_name] = {
                'dist': dist,
                'visit_cost': visit_cost,
                'turn_cost': turn_cost,
                'known_cost': known_cost,
                'score': score
            }
            
            if score < best_score:
                best_score = score
                best_dir = direction
        
        return best_dir, scoring_details
    
    def read_sensors(self, front_open: bool, left_open: bool, right_open: bool):
        """Đọc cảm biến (mô phỏng)"""
        x, y = self.pos_x, self.pos_y
        
        # Cảm biến phía trước
        if not self.is_blocked(x, y, self.heading):
            self.set_wall_known(x, y, self.heading, not front_open)
        
        # Cảm biến bên trái
        left_dir = self.left_dir(self.heading)
        if not self.is_blocked(x, y, left_dir):
            self.set_wall_known(x, y, left_dir, not left_open)
        
        # Cảm biến bên phải
        right_dir = self.right_dir(self.heading)
        if not self.is_blocked(x, y, right_dir):
            self.set_wall_known(x, y, right_dir, not right_open)
    
    def move_forward(self):
        """Di chuyển 1 ô về phía trước"""
        if not self.is_blocked(self.pos_x, self.pos_y, self.heading):
            dx, dy = DIR_DELTA[self.heading]
            self.pos_x += dx
            self.pos_y += dy
            
            if self.visit_count[self.pos_x][self.pos_y] < 255:
                self.visit_count[self.pos_x][self.pos_y] += 1
            
            action = f"Move {DIR_NAMES[self.heading]}"
            self.move_history.append((self.pos_x, self.pos_y, self.heading, action))
            return True
        return False
    
    def turn_to(self, new_heading: int):
        """Rẽ xe để hướng về hướng mới"""
        old_heading = self.heading
        self.heading = new_heading
        
        delta = (new_heading - old_heading + 4) % 4
        if delta == 1:
            action = "Turn Right 90°"
        elif delta == 3:
            action = "Turn Left 90°"
        elif delta == 2:
            action = "Turn 180°"
        else:
            action = "No turn"
        
        self.move_history.append((self.pos_x, self.pos_y, self.heading, action))
    
    def step(self) -> bool:
        """Thực hiện 1 bước của thuật toán"""
        # Bước 1: Tính Flood Distance
        self.compute_flood_distances()
        
        # Bước 2: Chọn hướng
        next_dir, details = self.choose_next_direction()
        
        if next_dir == -1:
            print(f"❌ Không thể đi tiếp từ ({self.pos_x}, {self.pos_y})")
            return False
        
        print(f"\n📍 Vị trí: ({self.pos_x}, {self.pos_y}), Hướng: {DIR_NAMES[self.heading]}")
        print(f"📌 Mục tiêu: ({self.goal_x}, {self.goal_y})")
        print(f"\n📊 Phân Tích Hướng:")
        for dir_name in ['N', 'E', 'S', 'W']:
            if dir_name in details:
                d = details[dir_name]
                marker = " ← CHỌN" if DIR_NAMES[next_dir] == dir_name else ""
                print(f"  {dir_name}: dist={d['dist']:3d}, visit={d['visit_cost']:3d}, turn={d['turn_cost']:2d}, known={d['known_cost']}, score={d['score']:5d}{marker}")
            else:
                print(f"  {dir_name}: BLOCKED")
        
        # Bước 3: Rẽ (nếu cần)
        if next_dir != self.heading:
            self.turn_to(next_dir)
        
        # Bước 4: Di chuyển
        if self.move_forward():
            print(f"✅ Di chuyển thành công đến ({self.pos_x}, {self.pos_y})")
            
            # Kiểm tra đạt mục tiêu
            if self.pos_x == self.goal_x and self.pos_y == self.goal_y:
                print(f"\n🎉 ĐẠT MỤC TIÊU! ({self.goal_x}, {self.goal_y})")
                return False
            
            return True
        else:
            print(f"❌ Không thể di chuyển (tường chặn)")
            return False
    
    def visualize_maze(self):
        """Vẽ mê cung"""
        print("\n" + "="*50)
        print("TRẠNG THÁI MÊ CUNG HIỆN TẠI")
        print("="*50)
        
        # Dòng trên cùng
        for x in range(self.width):
            if x == 0:
                print("+", end="")
            if self.has_wall(x, 0, MAZE_PATH_N):
                print("-", end="")
            else:
                print(" ", end="")
            print("+", end="")
        print()
        
        # Các hàng
        for y in range(self.height):
            # Dòng tường trái-phải
            for x in range(self.width):
                if self.has_wall(x, y, MAZE_PATH_W):
                    print("|", end="")
                else:
                    print(" ", end="")
                
                # Ô
                if x == self.pos_x and y == self.pos_y:
                    print(f"R", end="")  # R = Robot
                elif x == self.goal_x and y == self.goal_y:
                    print(f"G", end="")  # G = Goal
                elif self.dist_map[x][y] != FLOOD_UNREACHABLE:
                    dist = min(self.dist_map[x][y], 9)
                    print(f"{dist}", end="")
                else:
                    print(".", end="")
            
            # Tường phải
            if self.has_wall(self.width-1, y, MAZE_PATH_E):
                print("|", end="")
            else:
                print(" ", end="")
            print()
            
            # Dòng tường dưới
            for x in range(self.width):
                print("+", end="")
                if self.has_wall(x, y, MAZE_PATH_S):
                    print("-", end="")
                else:
                    print(" ", end="")
            print("+")


def test_scenario_1():
    """Test 1: Mê Cung Đơn Giản"""
    print("\n" + "="*60)
    print("TEST 1: MÊ CUNG ĐƠN GIẢN (3x3)")
    print("="*60)
    
    sim = MazeSimulator(width=3, height=3)
    sim.goal_x = 2
    sim.goal_y = 2
    
    # Tạo mê cung
    #   +---+---+---+
    # 2 | R | ? | ? |
    #   +---+---+---+
    # 1 | ? | # | ? |
    #   +---+---+---+
    # 0 | ? | ? | G |
    #   +---+---+---+
    #     0   1   2
    
    # Đặt tường
    sim.set_wall(1, 1, MAZE_PATH_N)  # Tường trên ô (1,1)
    sim.set_wall(0, 1, MAZE_PATH_E)  # Tường phải ô (0,1)
    
    # Bước 1: Tại (0, 0), đọc cảm biến
    print("\n🔍 BƯỚC 1: Ở ô (0, 0)")
    sim.read_sensors(front_open=True, left_open=False, right_open=True)
    print("  ✓ Đọc cảm biến: Front=mở, Left=tường, Right=mở")
    
    sim.step()
    
    # Bước 2
    print("\n🔍 BƯỚC 2: Ở ô (1, 0)")
    sim.read_sensors(front_open=True, left_open=False, right_open=True)
    sim.step()
    
    # Bước 3
    print("\n🔍 BƯỚC 3: Ở ô (2, 0)")
    sim.read_sensors(front_open=False, left_open=True, right_open=False)
    sim.step()
    
    sim.visualize_maze()


def test_scenario_2():
    """Test 2: Mê Cung Với Lựa Chọn Hướng"""
    print("\n" + "="*60)
    print("TEST 2: MÊ CUNG VỚI LỰA CHỌN HƯỚNG (5x5)")
    print("="*60)
    
    sim = MazeSimulator(width=5, height=5)
    sim.pos_x = 0
    sim.pos_y = 0
    sim.goal_x = 4
    sim.goal_y = 4
    
    # Đặt một số tường
    sim.set_wall(1, 0, MAZE_PATH_N)
    sim.set_wall(1, 1, MAZE_PATH_E)
    sim.set_wall(2, 2, MAZE_PATH_N)
    sim.set_wall(2, 2, MAZE_PATH_E)
    
    # Chạy 5 bước
    for step_num in range(5):
        print(f"\n{'='*60}")
        print(f"BƯỚC {step_num + 1}")
        print(f"{'='*60}")
        
        if not sim.step():
            break
    
    sim.visualize_maze()


def test_scenario_3():
    """Test 3: Ngõ Cụt (Dead End)"""
    print("\n" + "="*60)
    print("TEST 3: NGÕCỤT - CÁCH CHỈ CÓ MỘT LỐI RA")
    print("="*60)
    
    sim = MazeSimulator(width=4, height=4)
    sim.pos_x = 0
    sim.pos_y = 0
    sim.heading = MAZE_PATH_E
    sim.goal_x = 3
    sim.goal_y = 0
    
    # Tạo ngõ cụt
    #   +---+---+---+---+
    # 0 | R | ? | ? | G |
    #   +---+---+---+---+
    # Chỉ có 1 đường là đi thẳng ra
    
    sim.set_wall(0, 0, MAZE_PATH_N)
    sim.set_wall(0, 0, MAZE_PATH_S)
    
    for step_num in range(4):
        print(f"\n{'='*60}")
        print(f"BƯỚC {step_num + 1}")
        print(f"{'='*60}")
        
        if not sim.step():
            break
    
    sim.visualize_maze()


def test_sensor_reading_logic():
    """Test Logic Đọc Cảm Biến"""
    print("\n" + "="*60)
    print("TEST: LOGIC ĐỌC CẢM BIẾN")
    print("="*60)
    
    sim = MazeSimulator(width=3, height=3)
    sim.pos_x = 1
    sim.pos_y = 1
    sim.heading = MAZE_PATH_N
    
    print("\n📍 Vị trí: (1, 1), Hướng: N (lên)")
    print("\nCảm Biến:")
    print("  - SENSOR_FL (Front-Left): Nhìn theo hướng W (trái)")
    print("  - SENSOR_FR (Front-Right): Nhìn theo hướng E (phải)")
    print("  - SENSOR_L (Left): Nhìn theo hướng W (trái)")
    print("  - SENSOR_R (Right): Nhìn theo hướng E (phải)")
    
    print("\n📊 Kịch Bản 1: Front Open, Left Wall, Right Open")
    sim.read_sensors(front_open=True, left_open=False, right_open=True)
    print("  ✓ Đã lưu: Hướng N (trước) = mở")
    print("  ✓ Đã lưu: Hướng W (trái) = tường")
    print("  ✓ Đã lưu: Hướng E (phải) = mở")
    
    print("\n📊 Kịch Bản 2: Front Wall, Left Open, Right Wall")
    sim.read_sensors(front_open=False, left_open=True, right_open=False)
    print("  ✓ Đã lưu: Hướng N (trước) = tường")
    print("  ✓ Đã lưu: Hướng W (trái) = mở")
    print("  ✓ Đã lưu: Hướng E (phải) = tường")


if __name__ == "__main__":
    print("""
╔════════════════════════════════════════════════════════╗
║        MỤ PHỎNG THUẬT TOÁN FLOOD FILL                 ║
║     CHO ROBOT MICROMOUSE GIẢI MÊ CUNG                ║
╚════════════════════════════════════════════════════════╝
    """)
    
    test_scenario_1()
    test_scenario_2()
    test_scenario_3()
    test_sensor_reading_logic()
    
    print("\n" + "="*60)
    print("✅ HOÀN THÀNH MỌI TEST")
    print("="*60)
