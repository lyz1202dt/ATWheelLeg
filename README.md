# AT Wheel-Leg

AT Wheel-Leg 是一个面向轮腿平衡车的控制软件工程，当前包含：

- 可跨平台复用的轮腿控制核心；
- 面向 STM32H723 的嵌入式固件与硬件抽象层；
- 基于 ROS 2、MuJoCo 和 `mujoco_ros2_control` 的仿真控制器。

控制器以六个执行器为接口：左右两侧各两个腿部髋关节电机，以及左右轮电机。核心控制逻辑在 `Core` 中维护，仿真端和 MCU 端分别提供传感器、电机与总线的适配实现。

## 功能概览

### 控制核心

`Core` 提供与平台无关的 C++17 控制库，主要功能包括：

- 五连杆腿的正运动学与逆运动学；
- 基于自动微分计算雅可比矩阵；
- 腿端速度和力与关节速度、力矩之间的正逆映射；
- 随腿长线性插值的 LQR 增益调度；
- VMC 腿长控制；
- 左右腿角度同步与轮速差控制；
- 离地、恢复、平衡和 VMC 测试状态机；
- 传感器或运动学数据异常时的安全停机指令。

控制器的主要状态为：

| 状态 | 值 | 说明 |
| --- | ---: | --- |
| `VmcTest` | 0 | VMC 测试模式 |
| `Recovery` | 1 | 恢复到指定腿长和角度 |
| `Balance` | 2 | 接触地面后的平衡控制 |
| `Airborne` | 3 | 离地状态控制 |

未指定强制模式时，状态机会根据机身俯仰角和左右腿法向接触力自动切换。控制器默认使用 6 维状态：

```text
[轮位移, 轮速度, 机身俯仰+腿角, 其速度, -机身俯仰, -机身俯仰角速度]
```

### MCU 固件

`MCU` 是 STM32H723 固件工程，使用 STM32 HAL、FreeRTOS 和 CMSIS-DSP。当前工程包含：

- BMI088 IMU 驱动适配；
- SPI 中断/DMA 事务队列；
- FDCAN 收发与回调封装；
- UART、ADC、定时器、PWM 和 GPIO 封装；
- USB CDC 设备；
- MCU 端的 IMU、控制和测试任务入口；
- 预置的 17 个腿长采样点、每个采样点 12 个 LQR 增益值。

硬件初始化入口为 `bsp::hardware::init()`，应用入口为 `app_main()`。主循环由 FreeRTOS 调度，当前控制任务按 2 ms 周期调用公共控制器。

> 当前 MCU 应用中的六个 `Motor*` 仍是空指针占位，实际 DM4310 或其他电机驱动适配尚未接入控制任务。接入真实硬件前，需要实现 `Motor` 接口，并在 `MCU/APP/app_mian.cpp` 中完成电机对象初始化。

### MuJoCo 仿真

`MujocoSim` 是 ROS 2 工作空间，包含三个包：

- `car`：轮腿车 MuJoCo 模型、场景和安装资源；
- `controller`：LQR 控制器、MuJoCo 中间控制器、虚拟 IMU 和虚拟电机；
- `mujoco_ros2_control`：MuJoCo 与 ROS 2 Control 的硬件接口和传感器支持。

仿真控制器通过 `controller_manager` 加载：

- `lqr_controller/LQRController`：读取 IMU、关节状态和 `/cmd_vel`，调用 `Core::Controller` 生成六个执行器的力矩；
- `lqr_controller/MujocoSimController`：将控制器输出转换为 MuJoCo 侧的目标接口。

默认仿真控制频率为 500 Hz，LQR 增益表覆盖腿长 `0.18 m` 到 `0.34 m`，步长 `0.01 m`。

## 目录结构

```text
.
├── Core/                         # 跨平台控制核心
│   ├── inc/                      # 控制器、运动学、IMU/电机抽象和 LQR 接口
│   ├── src/                      # 控制器、腿部计算、增益调度实现
│   └── CMakeLists.txt
├── MCU/                          # STM32H723 固件
│   ├── APP/                      # 应用任务、BMI088 适配、LQR 增益表
│   ├── BSP/                      # SPI、FDCAN、UART、ADC、定时器、GPIO 等封装
│   ├── Core/                     # STM32CubeMX 生成的启动和外设代码
│   ├── Drivers/                 # STM32 HAL、CMSIS、FreeRTOS 和 USB 依赖
│   ├── CMakeLists.txt
│   └── CMakePresets.json
├── MujocoSim/
│   └── scr/
│       ├── car/                  # MuJoCo 模型和场景
│       ├── controller/           # ROS 2 控制器和配置
│       └── mujoco_ros2_control/  # MuJoCo ROS 2 Control 后端
├── third_party/
│   ├── autodiff/                 # 腿部雅可比矩阵自动微分
│   └── TinyMPC/                  # Eigen 备用头文件来源
├── DM4310.pdf                    # DM4310 电机资料
└── README.md
```

## 依赖

### Core

- CMake 3.16 或更高版本；
- 支持 C++17 的 GCC 或 Clang；
- Eigen3；
- 仓库内的 `third_party/autodiff`。

Core 会优先使用系统 Eigen3；如果找不到，则回退到 `third_party/TinyMPC/include/Eigen`。

### MCU

- CMake 3.22 或更高版本；
- Ninja；
- `arm-none-eabi-gcc` 工具链；
- STM32H723 目标环境；
- 仓库内的 STM32 HAL、FreeRTOS、CMSIS 和 USB Device Library。

### MuJoCo 仿真

- ROS 2；
- `colcon`；
- `controller_manager`、`controller_interface`、`hardware_interface`；
- `xacro`、`urdf`、`geometry_msgs`、`sensor_msgs`；
- MuJoCo、GLFW；
- `mujoco_ros2_control` 所需的 ROS 2 和视觉相关依赖。

## 构建与测试

### 构建 Core 并运行检查

在项目根目录执行：

```bash
cmake -S Core -B Core/build -G Ninja -DCORE_BUILD_TESTS=ON
cmake --build Core/build
ctest --test-dir Core/build --output-on-failure
```

测试程序 `core_test` 会验证：

- 腿部逆运动学与正运动学的往返误差；
- LQR 增益表的数据量、有限性和长度配置校验。

### 构建 STM32 固件

需要先确认 `arm-none-eabi-gcc` 已加入 `PATH`，然后执行：

```bash
cd MCU
cmake --preset Debug
cmake --build --preset Debug
```

Release 构建：

```bash
cd MCU
cmake --preset Release
cmake --build --preset Release
```

构建目录为 `MCU/build/Debug` 或 `MCU/build/Release`，目标程序名为 `wheel_leg`，通常可在对应目录中找到 `.elf` 和 `.map` 文件。

### 构建 ROS 2 仿真工作空间

从项目根目录执行：

```bash
source /opt/ros/<ros2-distro>/setup.bash
cd MujocoSim
colcon build --symlink-install --base-paths scr
source install/setup.bash
```

如果系统中已有可用的 MuJoCo ROS 2 Control 依赖，也可以只重新构建本项目控制器：

```bash
colcon build --symlink-install --base-paths scr \
  --packages-select car controller mujoco_ros2_control
```

## 运行仿真

构建并 source 工作空间后：

```bash
ros2 launch controller sim_controller.launch.py \
  show_gui:=true \
  simulation_frequency:=500.0 \
  realtime_factor:=1.0
```

发送前进速度和转向角速度：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}" -r 20
```

其中：

- `linear.x` 对应期望前进速度；
- `angular.z` 对应期望偏航角速度；
- `show_gui` 控制 MuJoCo 图形界面；
- `simulation_frequency` 控制仿真更新频率；
- `realtime_factor` 控制仿真相对实时速度。

主要仿真配置位于 [`MujocoSim/scr/controller/config/sim_controller.yaml`](MujocoSim/scr/controller/config/sim_controller.yaml)，包括 VMC 参数、状态模式和全部 LQR 增益表。

## 关键接口

公共控制器的最小使用方式如下：

```cpp
Controller controller(imu, lf, rf, lb, rb, lw, rw);

Controller::Params params;
controller.set_params(params);

std::string error;
controller.set_gain_table(gain_lengths, gain_values, error);

controller.input(expected_velocity, expected_omega, expected_leg_length, mode);
controller.update(dt);
```

传感器和执行器需要分别实现：

- `IMUBase`：提供姿态、角速度、加速度和有效状态；
- `Motor`：提供状态读取以及位置、速度、力矩、`kp`、`kd` 指令接口。

## 安全与参数说明

- 控制器会检查时间步长、IMU 状态、姿态四元数、运动学结果和增益表数据；
- 输入或状态无效时，会向六个电机发送零指令；
- 髋关节输出力矩在公共控制器中限制为 `±12 N·m`；
- 轮电机输出力矩在公共控制器中限制为 `±2 N·m`；
- 仿真侧还通过 `effort_limit` 对执行器输出进行限制；
- 修改腿长、连杆长度、轮半径、LQR 增益或 VMC 参数后，应同时检查仿真模型、MCU 增益表和控制器参数的一致性。

## 相关资料

- [DM4310 电机资料](DM4310.pdf)
- [Core CMake 配置](Core/CMakeLists.txt)
- [MCU CMake Presets](MCU/CMakePresets.json)
- [仿真启动文件](MujocoSim/scr/controller/launch/sim_controller.launch.py)
- [仿真控制参数](MujocoSim/scr/controller/config/sim_controller.yaml)
