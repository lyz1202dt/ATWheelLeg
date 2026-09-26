# AT Wheel-Leg

AT Wheel-Leg 是一个面向轮腿平衡车的控制软件工程，当前包含：

- 可跨平台复用的轮腿控制核心；
- 面向 STM32H723 的嵌入式固件与硬件抽象层；
- 基于 ROS 2、MuJoCo 和 `mujoco_ros2_control` 的仿真控制器。

控制器以六个执行器为接口：左右两侧各两个腿部髋关节电机，以及左右轮电机。核心控制逻辑在 `Core` 中维护，所有控制器继承自 `ControllerBase` 抽象基类，仿真端和 MCU 端分别提供传感器、电机与总线的适配实现。

## 功能概览

### 控制核心

`Core` 提供与平台无关的 C++17 控制库。所有控制器都继承自 `ControllerBase`，持有六个 `Motor*` 和一个 `IMUBase*`，并通过统一的 `input()` / `update()` 接口被调用。当前内置两套控制器：

- `Controller`：6 状态 LQR/VMC 平衡控制器（用于平衡小车 `car` 与 MCU 固件）；
- `ControllerAT`：10 状态全身（WBC）LQR 控制器（用于 AT 轮腿机器人）。

主要功能包括：

- 三种腿部构型的正运动学、逆运动学与雅可比矩阵：
  - `LegCalc`：五连杆并联结构；
  - `LegCalc2`：偏置并联结构；
  - `LegCalc3`：真串联结构；
- 基于自动微分计算雅可比矩阵，并完成腿端速度/力与关节速度/力矩之间的正逆映射；
- 通用 `PID` 控制器（普通、微分先行等）；
- 随腿长线性插值的 LQR 增益调度，支持增益表 `k_tab` 与在线求解两种模式；
- VMC 腿长控制、左右腿角度同步与轮速差控制；
- 离地、恢复、平衡和 VMC 测试状态机；
- 传感器或运动学数据异常时的安全停机指令。

`Controller`（6 状态 LQR/VMC）的主要状态为：

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

`ControllerAT`（10 状态全身 LQR）的状态机为：

| 状态 | 说明 |
| --- | --- |
| `IDEL` | 位控处于默认站姿 |
| `KINAMIC_TEST` | 运动学测试 |
| `VMC_TEST` | 测试 VMC 功能 |
| `READY_STAND1` | 斜坡渐变当前期望到准备站立的姿态 |
| `READY_STAND2` | 完成小板凳形态 |
| `TOUCH_GROUND` | 接触地面，LQR 平衡控制 |
| `LEFT_FLY` | 左侧悬空，LQR 左侧失控 |
| `RIGHT_FLAY` | 右侧悬空，LQR 右侧失控 |
| `ALL_FLY` | 全部悬空，LQR 仅姿态稳定 |

### MCU 固件

`MCU` 是 STM32H723 固件工程，使用 STM32 HAL、FreeRTOS 和 CMSIS-DSP。当前工程包含：

- BMI088 IMU 驱动适配；
- SPI 中断/DMA 事务队列；
- FDCAN 收发与回调封装；
- UART、ADC、定时器、PWM 和 GPIO 封装；
- USB CDC 设备；
- MCU 端的 IMU、电机、控制、LQR 在线调试和测试任务；
- 预置的 17 个腿长采样点、每个采样点 12 个 LQR 增益值。

硬件初始化入口为 `bsp::hardware::init()`，应用入口为 `app_main()`。主循环由 FreeRTOS 调度，应用会创建以下任务：

- `imu_task`：以 1 ms 周期更新 IMU；
- `motor_task`：以 1 ms 周期预留电机收发（当前为空占位）；
- `control_task`：以 2 ms 周期调用 `Controller::update()`；
- `lqr_task`：以 100 ms 周期轮询在线 LQR 权重更新请求，通过 `lqr_q_diag` / `lqr_r_diag` 调试变量在线求解并更新增益；
- `test_task`：测试任务入口。

> 当前 MCU 应用中的六个 `Motor*` 仍是空指针占位，实际 DM4310 或其他电机驱动适配尚未接入控制任务。接入真实硬件前，需要实现 `Motor` 接口，并在 `MCU/APP/app_main.cpp` 中完成电机对象初始化。

### MuJoCo 仿真

`MujocoSim` 是 ROS 2 工作空间，包含五个包：

- `car`：轮腿平衡小车 MuJoCo 模型、场景和安装资源；
- `atwl`：AT 轮腿机器人 MuJoCo 模型（STL 网格）、场景与物理参数表；
- `wl`：另一款轮腿机器人（COD-2026RoboMaster-Balance）模型；
- `controller`：LQR 控制器、AT 控制器、MuJoCo 中间控制器、虚拟 IMU 和虚拟电机；
- `mujoco_ros2_control`：MuJoCo 与 ROS 2 Control 的硬件接口和传感器支持。

`controller` 包通过 `controller_manager` 提供三个插件：

- `lqr_controller/LQRController`：读取 IMU、关节状态和 `/cmd_vel`，调用 `Core::Controller` 生成六个执行器的力矩；
- `lqr_controller/LQRControllerAT`：读取 IMU、关节状态和 `/cmd_vel`，调用 `Core::ControllerAT`（10 状态全身 LQR）控制 AT 轮腿机器人；
- `lqr_controller/MujocoSimController`：链式中间控制器，将上层控制器输出转换为 MuJoCo 侧的位置/速度/力矩/`kp`/`kd` 目标接口。

默认仿真控制频率为 500 Hz。`sim_controller.yaml` 中 LQR 增益表覆盖腿长 `0.18 m` 到 `0.34 m`，步长 `0.01 m`。

## 目录结构

```text
.
├── Core/                         # 跨平台控制核心
│   ├── inc/                      # 控制器、运动学、IMU/电机抽象、PID 和 LQR 接口
│   ├── src/                      # 控制器、腿部计算、PID、增益调度与测试实现
│   ├── cmake/                    # Core CMake 包导出配置
│   └── CMakeLists.txt
├── MCU/                          # STM32H723 固件
│   ├── APP/                      # 应用任务、BMI088 适配、LQR 增益表
│   ├── BSP/                      # SPI、FDCAN、UART、ADC、定时器、GPIO 等封装
│   ├── Core/                     # STM32CubeMX 生成的启动和外设代码
│   ├── Drivers/                  # STM32 HAL、CMSIS、FreeRTOS 和 USB 依赖
│   ├── CMakeLists.txt
│   └── CMakePresets.json
├── MujocoSim/
│   └── scr/
│       ├── car/                  # 平衡小车 MuJoCo 模型和场景
│       ├── atwl/                 # AT 轮腿机器人模型、场景和物理参数
│       ├── wl/                   # 另一款轮腿机器人模型
│       ├── controller/           # ROS 2 控制器、配置、启动文件和 urdf
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

- `LegCalc2`（偏置并联）逆运动学与正运动学的往返误差；
- `LegCalc`（五连杆并联）与 `LegCalc3`（真串联）的正逆运动学往返及零角度约定；
- 速度映射与雅可比矩阵的有限差分一致性；
- LQR 增益的在线求解。

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
  --packages-select car atwl wl controller mujoco_ros2_control
```

## 运行仿真

构建并 source 工作空间后，可分别运行平衡小车与 AT 轮腿机器人两套仿真。

### 平衡小车（car）

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

### AT 轮腿机器人（atwl）

```bash
ros2 launch controller atwl_controller.launch.py \
  show_gui:=true \
  simulation_frequency:=500.0 \
  realtime_factor:=1.0
```

其中：

- `linear.x` 对应期望前进速度；
- `angular.z` 对应期望偏航角速度；
- `show_gui` 控制 MuJoCo 图形界面；
- `simulation_frequency` 控制仿真更新频率；
- `realtime_factor` 控制仿真相对实时速度。

主要仿真配置位于：

- 平衡小车：[`MujocoSim/scr/controller/config/sim_controller.yaml`](MujocoSim/scr/controller/config/sim_controller.yaml)，包括 VMC 参数、状态模式和全部 LQR 增益表；
- AT 轮腿机器人：[`MujocoSim/scr/controller/config/atwl_controller.yaml`](MujocoSim/scr/controller/config/atwl_controller.yaml)，包括关节名称、IMU 话题、`/cmd_vel` 话题与力矩限制。

## 关键接口

所有控制器继承自 `ControllerBase`，最小使用方式如下：

```cpp
Controller controller(imu, lf, rf, lb, rb, lw, rw);

Controller::Params params;
controller.set_params(params);

std::string error;
controller.set_gain_table(gain_lengths, gain_values, error);

controller.input(expected_velocity, expected_omega, expected_leg_length, mode);
controller.update(dt);
```

AT 轮腿机器人使用 `ControllerAT`，通过 `ControllerBase*` 指针即可在仿真包装层中替换具体实现：

```cpp
ControllerAT controller_at(imu, lf, rf, lb, rb, lw, rw);
ControllerBase* controller = &controller_at;

controller->input(velocity, omega, height, mode);
controller->update(dt);
```

传感器和执行器需要分别实现：

- `IMUBase`：提供姿态、角速度、加速度和有效状态；
- `Motor`：提供状态读取以及位置、速度、力矩、`kp`、`kd` 指令接口。

## 安全与参数说明

- 控制器会检查时间步长、IMU 状态、姿态四元数、运动学结果和增益表数据；
- 输入或状态无效时，会向六个电机发送零指令；
- 髋关节输出力矩在公共控制器中限制为 `±12 N·m`；
- 轮电机输出力矩在公共控制器中限制为 `±2 N·m`；
- 仿真侧还通过 `effort_limit` 对执行器输出进行限制（默认 `20 N·m`）；
- `ControllerAT` 支持在线更新 LQR 权重，MCU 端对应 `lqr_q_diag` / `lqr_r_diag` 调试变量；
- 修改腿长、连杆长度、轮半径、LQR 增益或 VMC 参数后，应同时检查仿真模型、MCU 增益表和控制器参数的一致性。

## 相关资料

- [DM4310 电机资料](DM4310.pdf)
- [Core CMake 配置](Core/CMakeLists.txt)
- [MCU CMake Presets](MCU/CMakePresets.json)
- [平衡小车启动文件](MujocoSim/scr/controller/launch/sim_controller.launch.py)
- [AT 轮腿机器人启动文件](MujocoSim/scr/controller/launch/atwl_controller.launch.py)
- [平衡小车控制参数](MujocoSim/scr/controller/config/sim_controller.yaml)
- [AT 轮腿机器人控制参数](MujocoSim/scr/controller/config/atwl_controller.yaml)
