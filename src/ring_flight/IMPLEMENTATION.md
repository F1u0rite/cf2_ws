# Ring Flight 项目实现文档

## 1. 系统架构

### 1.1 整体架构

```
┌─────────────────────────────────────────────────────────────┐
│                     应用层 (Python)                           │
│  ring_flight.py - 主控脚本                                    │
│  ├── 状态机管理                                              │
│  ├── 路径规划器                                              │
│  └── 安全监控器                                              │
├─────────────────────────────────────────────────────────────┤
│  Crazyswarm2 中间层 (crazyflie_py)                           │
│  ├── Crazyswarm 类 - 初始化和管理                            │
│  ├── Crazyflie 类 - 单机控制 API                             │
│  │   ├── takeoff(targetHeight, duration)                    │
│  │   ├── land(targetHeight, duration)                       │
│  │   ├── goTo(goal, yaw, duration)                          │
│  │   └── uploadTrajectory(...)                              │
│  └── TimeHelper - 时间同步                                   │
├─────────────────────────────────────────────────────────────┤
│  ROS 2 通信层                                                 │
│  ├── crazyflie_server (C++/Python)                          │
│  ├── Services: Takeoff, Land, GoTo, UploadTrajectory        │
│  ├── Topics: /cf1/pose, /cf1/status, /cf1/odom              │
│  └── Crazyradio PA 通信                                      │
├─────────────────────────────────────────────────────────────┤
│  Crazyflie Firmware (嵌入式)                                 │
│  ├── High Level Commander (100 Hz)                          │
│  ├── Mellinger 控制器 (500 Hz)                              │
│  ├── 扩展卡尔曼滤波器 (500 Hz)                               │
│  └── 电机控制 (500 Hz)                                       │
├─────────────────────────────────────────────────────────────┤
│  硬件层                                                       │
│  ├── Crazyflie 2.1                                           │
│  ├── Lighthouse V2 / Loco Positioning                       │
│  └── Crazyradio PA                                           │
└─────────────────────────────────────────────────────────────┘
```

### 1.2 Crazyswarm2 通信模式

**重要**: Crazyswarm2 使用服务调用模式，而非直接发布 topic。

```python
# ✅ 正确的方式
from crazyflie_py import Crazyswarm

swarm = Crazyswarm()
cf = swarm.allcfs.crazyflies[0]
cf.takeoff(targetHeight=1.0, duration=2.5)  # 内部调用 ROS 2 service
cf.goTo(goal, yaw, duration)                # 内部调用 ROS 2 service

# ❌ 错误的方式（不要这样做）
# from geometry_msgs.msg import Twist
# vel_pub = node.create_publisher(Twist, '/cf1/cmd_vel', 10)
# vel_pub.publish(twist_msg)  # Crazyswarm2 不使用这种模式
```

## 2. 核心模块设计

### 2.1 状态机模块

#### 状态定义
```python
from enum import Enum

class FlightState(Enum):
    IDLE = 0           # 空闲状态
    TAKING_OFF = 1     # 起飞中
    HOVERING = 2       # 悬停中
    PLANNING = 3       # 路径规划中
    FLYING_TO_RING = 4 # 飞向环
    PASSING_RING = 5   # 穿越环
    LANDING = 6        # 降落中
    FINISHED = 7       # 任务完成
```

#### 状态转换逻辑
```
IDLE -> TAKING_OFF -> HOVERING -> PLANNING -> FLYING_TO_RING
                                                   │
                                                   v
FINISHED <- LANDING <─────────────────────── PASSING_RING
                                                   │
                                                   v
                                            (下一个环或降落)
```

### 2.2 环位置管理模块

#### 数据结构
```python
from dataclasses import dataclass
import numpy as np

@dataclass
class Ring:
    id: int
    position: np.ndarray  # [x, y, z] 环中心位置
    yaw: float  # 环的朝向（弧度）
    radius: float  # 环的半径
    passed: bool = False  # 是否已穿过
    
    def get_normal(self):
        """计算环平面的法向量"""
        return np.array([
            np.cos(self.yaw),
            np.sin(self.yaw),
            0.0
        ])
```

#### 配置文件
```yaml
# config/rings.yaml
rings:
  - id: 1
    position: [1.0, 0.0, 1.0]  # x, y, z (米)
    yaw: 0.0  # 弧度，0 表示朝向 +X 方向
    radius: 0.25  # 米
  - id: 2
    position: [2.0, 1.0, 1.0]
    yaw: 1.57  # 90度，朝向 +Y 方向
    radius: 0.25
```

### 2.3 路径规划模块

#### 规划策略
使用简单的直线接近法（推荐初期使用）：

1. **预接近点**: 环前方 0.5-0.8 米
2. **环中心点**: 穿越目标
3. **后接近点**: 环后方 0.3-0.5 米

```python
def plan_path_to_ring(current_pos, ring, approach_dist=0.7, exit_dist=0.4):
    """规划到环的路径
    
    Args:
        current_pos: 当前位置 [x, y, z]
        ring: Ring 对象
        approach_dist: 预接近距离（米）
        exit_dist: 穿出距离（米）
    
    Returns:
        waypoints: 路径点列表
    """
    normal = ring.get_normal()
    
    # 预接近点：环前方
    pre_approach = ring.position - normal * approach_dist
    
    # 环中心点
    ring_center = ring.position.copy()
    
    # 后接近点：环后方
    post_approach = ring.position + normal * exit_dist
    
    waypoints = [
        current_pos,
        pre_approach,
        ring_center,
        post_approach
    ]
    
    return waypoints
```

### 2.4 安全监控模块

```python
class SafetyMonitor:
    def __init__(self):
        self.bounds = {
            'x': (-1.5, 1.5),
            'y': (-1.5, 1.5),
            'z': (0.2, 2.0)
        }
        self.battery_critical = 3.7  # V
        self.battery_warning = 3.8   # V
    
    def check_waypoint_safe(self, waypoint):
        """检查路径点是否在安全边界内"""
        x, y, z = waypoint
        if not (self.bounds['x'][0] <= x <= self.bounds['x'][1]):
            return False, f"X out of bounds: {x}"
        if not (self.bounds['y'][0] <= y <= self.bounds['y'][1]):
            return False, f"Y out of bounds: {y}"
        if not (self.bounds['z'][0] <= z <= self.bounds['z'][1]):
            return False, f"Z out of bounds: {z}"
        return True, "OK"
    
    def check_battery(self, voltage):
        """检查电池状态"""
        if voltage < self.battery_critical:
            return "CRITICAL", "Battery critical, land immediately!"
        elif voltage < self.battery_warning:
            return "WARNING", "Battery low"
        return "OK", "Battery OK"
```

## 3. 实现步骤

### 阶段 1: 基础飞行脚本

#### 步骤 1.1: 创建基础脚本
```python
# ring_flight/ring_flight.py
#!/usr/bin/env python3

from crazyflie_py import Crazyswarm
import numpy as np
import yaml
from pathlib import Path

class RingFlight:
    def __init__(self):
        # 初始化 Crazyswarm2
        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.cf = self.swarm.allcfs.crazyflies[0]
        
        # 加载环配置
        self.rings = self.load_rings('config/rings.yaml')
        
        # 飞行参数
        self.hover_height = 1.0
        self.approach_velocity = 0.3  # m/s
        self.passing_velocity = 0.4   # m/s
        
    def load_rings(self, config_file):
        """从配置文件加载环信息"""
        config_path = Path(__file__).parent / config_file
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        rings = []
        for ring_data in config['rings']:
            ring = Ring(
                id=ring_data['id'],
                position=np.array(ring_data['position']),
                yaw=ring_data['yaw'],
                radius=ring_data['radius']
            )
            rings.append(ring)
        return rings
    
    def run(self):
        """执行完整的穿环任务"""
        print("Starting ring flight mission...")
        
        # 起飞
        print("Taking off...")
        self.cf.takeoff(targetHeight=self.hover_height, duration=2.5)
        self.timeHelper.sleep(3.0)
        
        # 穿越每个环
        for ring in self.rings:
            print(f"Flying to ring {ring.id}...")
            self.fly_through_ring(ring)
        
        # 降落
        print("Landing...")
        self.cf.land(targetHeight=0.04, duration=2.5)
        self.timeHelper.sleep(3.0)
        
        print("Mission completed!")
    
    def fly_through_ring(self, ring):
        """穿越单个环"""
        # 获取当前位置
        current_pos = np.array(self.cf.position())
        
        # 规划路径
        waypoints = plan_path_to_ring(current_pos, ring)
        
        # 飞向预接近点
        print(f"  Approaching ring {ring.id}...")
        self.cf.goTo(waypoints[1], 0, duration=3.0)
        self.timeHelper.sleep(3.5)
        
        # 穿过环中心
        print(f"  Passing through ring {ring.id}...")
        self.cf.goTo(waypoints[2], 0, duration=2.0)
        self.timeHelper.sleep(2.5)
        
        # 飞到后接近点
        print(f"  Exiting ring {ring.id}...")
        self.cf.goTo(waypoints[3], 0, duration=1.5)
        self.timeHelper.sleep(2.0)
        
        ring.passed = True

def main():
    flight = RingFlight()
    flight.run()

if __name__ == '__main__':
    main()
```

#### 步骤 1.2: 更新 setup.py
```python
# setup.py
from setuptools import find_packages, setup

package_name = 'ring_flight'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/rings.yaml']),
        ('share/' + package_name + '/launch', ['launch/ring_flight.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yd',
    maintainer_email='ydd2844@gmail.com',
    description='Autonomous ring flight for Crazyflie',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'ring_flight = ring_flight.ring_flight:main'
        ],
    },
)
```

### 阶段 2: 配置文件

#### 步骤 2.1: 创建 rings.yaml
```yaml
# config/rings.yaml
rings:
  - id: 1
    position: [1.0, 0.0, 1.0]
    yaw: 0.0
    radius: 0.25
  - id: 2
    position: [2.0, 0.5, 1.0]
    yaw: 0.785  # 45度
    radius: 0.25
```

#### 步骤 2.2: 配置 crazyflies.yaml
这是 Crazyswarm2 最核心的配置文件，位于 `crazyflie/config/crazyflies.yaml`。

```yaml
# 修改 cf2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml

robots:
  cf1:  # 你的 Crazyflie 名称
    enabled: true
    uri: radio://0/80/2M/E7E7E7E701  # 修改为你的 Crazyflie URI
    initial_position: [0.0, 0.0, 0.0]
    type: cf21

robot_types:
  cf21:
    big_quad: false
    battery:
      voltage_warning: 3.8
      voltage_critical: 3.7

all:
  firmware_logging:
    enabled: true
    default_topics:
      pose:
        frequency: 10  # Hz
      status:
        frequency: 1   # Hz
  
  firmware_params:
    commander:
      enHighLevel: 1  # 启用 High Level Commander
    stabilizer:
      estimator: 2  # 2: Kalman filter
      controller: 2  # 2: Mellinger controller
    locSrv:
      extPosStdDev: 1e-3  # 外部定位标准差
  
  reference_frame: "world"
```

**重要参数说明**:
- `uri`: Crazyflie 的无线地址，可通过 cfclient 查看
- `enHighLevel: 1`: 必须启用，否则 `takeoff()`/`goTo()` 不工作
- `estimator: 2`: 使用扩展卡尔曼滤波器（推荐）
- `controller: 2`: 使用 Mellinger 控制器（推荐，性能优于 PID）

### 阶段 3: Launch 文件

#### 步骤 3.1: 创建 launch 文件
```python
# launch/ring_flight.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 参数
    backend = LaunchConfiguration('backend')
    
    backend_arg = DeclareLaunchArgument(
        'backend',
        default_value='cpp',  # 'cpp' 为实机模式，'sim' 为仿真模式
        description='Backend to use (cpp or sim)'
    )
    
    # 包含 Crazyswarm2 的 crazyflie server
    crazyflie_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('crazyflie'),
                'launch',
                'launch.py'
            )
        ]),
        launch_arguments={
            'backend': backend,
        }.items()
    )
    
    # ring_flight 节点
    ring_flight_node = Node(
        package='ring_flight',
        executable='ring_flight',
        name='ring_flight',
        output='screen'
    )
    
    return LaunchDescription([
        backend_arg,
        crazyflie_launch,
        ring_flight_node
    ])
```

## 4. 测试策略

### 4.1 仿真测试（推荐先做）

```bash
# 1. 启动仿真模式
ros2 launch ring_flight ring_flight.launch.py backend:=sim

# 仿真模式优点：
# - 无需实际硬件
# - 快速迭代
# - 安全测试路径规划逻辑
```

### 4.2 实机测试

```bash
# 1. 确保 Crazyradio PA 已连接
lsusb | grep 1915:7777

# 2. 确保 crazyflies.yaml 中 URI 正确

# 3. 启动实机模式
ros2 launch ring_flight ring_flight.launch.py backend:=cpp

# 4. 测试步骤：
#    a. 先测试起飞降落
#    b. 再测试单点飞行
#    c. 最后测试穿环
```

### 4.3 单元测试

```python
# test/test_path_planning.py
import unittest
import numpy as np
from ring_flight.ring_flight import Ring, plan_path_to_ring

class TestPathPlanning(unittest.TestCase):
    def test_waypoint_generation(self):
        ring = Ring(
            id=1,
            position=np.array([1.0, 0.0, 1.0]),
            yaw=0.0,
            radius=0.25
        )
        current_pos = np.array([0.0, 0.0, 1.0])
        
        waypoints = plan_path_to_ring(current_pos, ring)
        
        # 检查路径点数量
        self.assertEqual(len(waypoints), 4)
        
        # 检查预接近点在环前方
        self.assertLess(waypoints[1][0], ring.position[0])
        
        # 检查后接近点在环后方
        self.assertGreater(waypoints[3][0], ring.position[0])

if __name__ == '__main__':
    unittest.main()
```

## 5. 调试技巧

### 5.1 使用 RViz2 可视化

```bash
# 启动 RViz2
ros2 run rviz2 rviz2

# 添加以下 displays:
# - TF (查看坐标系)
# - PoseStamped (topic: /cf1/pose, 查看无人机位置)
# - Marker (可以发布环的位置标记)
```

### 5.2 查看日志

```bash
# 查看 Crazyflie 状态
ros2 topic echo /cf1/status

# 查看位置
ros2 topic echo /cf1/pose

# 查看所有 topic
ros2 topic list
```

### 5.3 记录飞行数据

```bash
# 录制 ROS 2 bag
ros2 bag record -a -o ring_flight_test

# 回放
ros2 bag play ring_flight_test
```

## 6. 常见问题和解决方案

### 问题 1: `cf.takeoff()` 无响应
**原因**: `enHighLevel` 未启用
**解决方案**: 在 `crazyflies.yaml` 中设置 `commander.enHighLevel: 1`

### 问题 2: 定位不稳定，无人机抖动
**原因**: 定位系统配置问题或环境干扰
**解决方案**:
- 检查 Lighthouse 基站是否稳固
- 调整 `locSrv.extPosStdDev` 参数
- 确保飞行区域无反光物体

### 问题 3: 穿环失败，撞到环
**原因**: 路径规划不准确或速度过快
**解决方案**:
- 增大 `approach_dist` (预接近距离)
- 降低飞行速度 (增大 `duration` 参数)
- 增大环的尺寸
- 手动测量并精确标定环位置

### 问题 4: 连接不上 Crazyflie
**原因**: URI 错误或 Crazyradio PA 问题
**解决方案**:
```bash
# 扫描 Crazyflie
ros2 run crazyflie scan

# 检查 USB 权限
sudo chmod 666 /dev/ttyUSB0
```

### 问题 5: 仿真模式下无法启动
**原因**: 缺少仿真依赖
**解决方案**:
```bash
# 安装仿真后端
sudo apt install ros-humble-tf-transformations
```

## 7. 性能优化

### 7.1 参数调优

```python
# 推荐的初始参数
PARAMS = {
    'hover_height': 1.0,        # 悬停高度
    'approach_distance': 0.7,   # 预接近距离
    'exit_distance': 0.4,       # 穿出距离
    'approach_duration': 3.0,   # 接近时间（秒）
    'passing_duration': 2.0,    # 穿越时间（秒）
    'exit_duration': 1.5,       # 穿出时间（秒）
}

# 速度计算：速度 = 距离 / 时间
# 例如：0.7m / 3.0s = 0.23 m/s
```

### 7.2 轨迹优化

对于更平滑的轨迹，可以使用 `uploadTrajectory()`:

```python
from crazyflie_py.uav_trajectory import Trajectory

# 加载预先生成的轨迹
traj = Trajectory()
traj.loadcsv('trajectory.csv')

# 上传并执行
cf.uploadTrajectory(0, 0, traj)
cf.startTrajectory(0, timescale=1.0)
```

## 8. 下一步改进

### 短期改进
- [ ] 添加实时轨迹可视化（RViz2 Marker）
- [ ] 实现电池监控和自动降落
- [ ] 添加更详细的日志记录
- [ ] 支持动态调整飞行参数

### 长期改进
- [ ] 集成 AI Deck 进行视觉检测
- [ ] 实现动态避障
- [ ] 支持多机协同穿环
- [ ] 使用机器学习优化轨迹

## 9. 参考资源

### 官方文档
- [Crazyswarm2 文档](https://imrclab.github.io/crazyswarm2/)
- [Crazyflie 固件文档](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/)
- [ROS 2 Humble 文档](https://docs.ros.org/en/humble/)

### 示例代码
- [Crazyswarm2 示例](https://github.com/IMRCLab/crazyswarm2/tree/main/crazyflie_examples)
- [本项目参考的示例](cf2_ws/install/crazyflie_examples/)

### 工具
- **RViz2**: 3D 可视化
- **rqt**: ROS 2 图形化工具
- **PlotJuggler**: 数据绘图
- **cfclient**: Crazyflie 官方客户端

### 社区
- [Bitcraze 论坛](https://forum.bitcraze.io/)
- [Crazyswarm2 GitHub Issues](https://github.com/IMRCLab/crazyswarm2/issues)
