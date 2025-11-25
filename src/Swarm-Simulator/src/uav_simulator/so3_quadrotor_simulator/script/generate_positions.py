#!/usr/bin/env python
import random
import os

def generate_positions(num_uavs=9, min_distance=1.5, x_range=(-10, 10), y_range=(-10, 10), z=1.0):
    """
    生成无人机位置，确保最小间距
    """
    random.seed()  # 使用系统时间作为随机种子
    positions = []
    max_attempts = 1000
    
    for i in range(num_uavs):
        attempts = 0
        
        while attempts < max_attempts:
            x = round(random.uniform(x_range[0], x_range[1]), 3)
            y = round(random.uniform(y_range[0], y_range[1]), 3)
            
            # 检查与已有位置的距离
            too_close = False
            for pos in positions:
                dx = pos[0] - x
                dy = pos[1] - y
                distance = (dx*dx + dy*dy)**0.5
                if distance < min_distance:
                    too_close = True
                    break
            
            if not too_close:
                positions.append((x, y, z))
                break
            
            attempts += 1
        
        if attempts == max_attempts:
            # 如果无法找到合适位置，使用随机位置
            print(f"Warning: Could not find position for UAV {i+2} with min distance {min_distance}m")
            x = round(random.uniform(x_range[0], x_range[1]), 3)
            y = round(random.uniform(y_range[0], y_range[1]), 3)
            positions.append((x, y, z))
    
    return positions

def main():
    try:
        # 生成9个无人机的位置（uav2到uav10）
        positions = generate_positions(num_uavs=9, min_distance=1.5)
        
        # 设置环境变量
        for i, (x, y, z) in enumerate(positions):
            uav_num = i + 2
            os.environ[f'UAV{uav_num}_X'] = str(x)
            os.environ[f'UAV{uav_num}_Y'] = str(y)
            os.environ[f'UAV{uav_num}_Z'] = str(z)
            print(f"Set UAV{uav_num}: x={x}, y={y}, z={z}")
            
    except Exception as e:
        print(f"Error generating positions: {e}")

if __name__ == "__main__":
    main()