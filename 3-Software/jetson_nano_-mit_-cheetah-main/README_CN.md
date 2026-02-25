# MIT Mini Cheetah Software (自制硬件适配版)

本项目是 MIT Mini Cheetah 软件仓库的自制硬件适配版本。以下文档记录了编译、配置及针对自制硬件（Wheeltec IMU、自制腿部结构、Logitech F710 手柄）的详细调试与适配过程。

## 1. 编译与运行指南

### 1.1 命令行编译
建议在项目根目录下的 `mc-build` 文件夹中进行构建。

```bash
cd mc-build
rm CMakeCache.txt  # 清理旧配置（如有必要）

# 配置项目
# -DMINI_CHEETAH_BUILD=TRUE: 构建 Mini Cheetah 版本
# -DJCQP_USE_AVX2=OFF: 关闭 x86 AVX2 优化，适配 ARM 架构（如 Jetson Nano/NX）
cmake -DMINI_CHEETAH_BUILD=TRUE -DJCQP_USE_AVX2=OFF ..

# 编译（根据 CPU 核心数调整 -j 参数）
make -j4
```

### 1.2 运行控制器
生成的控制器位于 `user/MIT_Controller` 目录下。需要使用 `sudo` 权限运行以访问硬件端口。

*   **真机模式**：
    ```bash
    sudo ./user/MIT_Controller/mit_ctrl m r f
    ```
*   **仿真模式**：
    ```bash
    sudo ./user/MIT_Controller/mit_ctrl m s
    ```

---

## 2. 开发环境配置 (VSCode Remote - SSH)

推荐使用 VSCode 进行远程开发与调试。

1.  本机安装 **Remote - SSH** 与 **CMake Tools** 扩展。
2.  通过 Remote-SSH 连接到目标主机（如 Jetson），打开项目目录。
3.  项目已预置 `.vscode/settings.json`，自动配置构建参数。
4.  **首次使用**：
    *   按 `F1` 或 `Ctrl+Shift+P`，输入 `CMake: Select Kit`，选择远程环境的编译器（如 `GCC for arm64`）。
    *   确保 `mc-build` 目录存在。
5.  **一键编译**：
    *   点击 VSCode 底部状态栏的 **Build** 按钮即可触发远程编译。
    *   如需清理缓存，使用命令 `CMake: Delete Cache and Reconfigure`。

**`.vscode/settings.json` 参考配置：**
```jsonc
{
  "cmake.buildDirectory": "${workspaceFolder}/mc-build",
  "cmake.generator": "Unix Makefiles",
  "cmake.configureArgs": [
    "-DMINI_CHEETAH_BUILD=TRUE",
    "-DJCQP_USE_AVX2=OFF"
  ],
  "cmake.buildArgs": [
    "-j4"
  ]
}
```

---

## 3. 硬件配置说明

*   **IMU**: 默认适配 **Wheeltec H30** 惯导模块。
    *   可通过环境变量调整串口：
        *   `H30_IMU_PORT` (默认 `/dev/ttyACM0`)
        *   `H30_IMU_BAUD` (默认 `460800`)
*   **遥控器**: 适配 **Logitech F710** 无线手柄。
    *   手柄必须拨至 **D (DirectInput)** 模式。
    *   默认设备路径：`/dev/input/js0` (可通过 `F710_DEVICE` 环境变量修改)。
    *   程序强制 `use_rc=0`，无需连接 SBUS 接收机。

---

## 4. 自制硬件适配与调试记录 (2025-11)

### 4.1 核心问题解决：自旋与侧翻
在调试初期，机器人进入 `BalanceStand` 后会出现剧烈的自旋（Yaw 轴）或侧翻（Roll 轴）。

*   **原因**：Wheeltec H30 IMU 的 Roll/Pitch/Yaw 三轴数据方向与 MIT 控制器预期**完全相反**。导致控制器输出的正反馈力矩加剧了失稳。
    *   现象：左转 Yaw 减小（应增加），右倾 Roll 减小（应增加），抬头 Pitch 增加（MIT 定义低头为正）。
*   **修复**：修改 `third-party/wheeltec_imu/WheeltecImu.cpp` 中的 `handlePacket` 函数。
    *   **操作**：将四元数转换为 RPY 欧拉角，**对 Roll, Pitch, Yaw 三轴全部取反**，再转回四元数。
    *   **结果**：姿态估计正确，`BalanceStand` 立即稳定。

### 4.2 动力学模型修正
*   **文件**：`common/include/Dynamics/MiniCheetah.h`
*   **腿部惯量**：自制腿部较重。尝试过放大惯量 (`inertia_scale = 2.0`)，但导致控制过激。最终**回调至 1.0** (MIT 原版参数)，配合增益调整效果最佳。
*   **机身惯量**：将人为放大的 `bodyRotationalInertia` 从 3.0 倍回调至 **1.5 倍**，使模型更接近物理真实。

### 4.3 底层驱动校准
*   **文件**：`robot/src/rt/rt_spi.cpp`
*   **电机零位**：硬编码的 `abad/hip/knee_offset` 是底层物理零位，必须严格对应电机编码器的几何零点。
*   **电机方向**：验证了 `*_side_sign` 数组。确认左后腿 (Leg 3) Abad 向外摆动时读数增加（正），符合 MIT 定义。

### 4.4 控制参数调优
*   **文件**：`config/mc-mit-ctrl-user-parameters.yaml`
*   **解决“趴得低”**：
    *   增加 Z 轴位置刚度 `Kp_body[2]`：从 60 提高到 **100**。
*   **解决“Hip 软 / 前后晃”**：
    *   增加关节位置刚度 `Kp_joint`：从 `[6, 6, 6]` 调整为 **`[6, 8, 8]`** (Abad, Hip, Knee)。
    *   增加关节阻尼 `Kd_joint`：从 `[0.4, 0.4, 0.4]` 调整为 **`[0.4, 0.8, 0.8]`**。
*   **解决“劈叉”**：
    *   主要依赖增加 Abad 关节刚度（如上所述）。
    *   如有需要，可降低 `RPC_mu` (摩擦系数) 来限制侧向力。

### 4.5 手柄控制功能实现
MIT 原始代码未将 Gamepad 数据映射到核心控制指令 `rcCommand`，导致手柄摇杆无效。

*   **修复**：在 `robot/src/RobotRunner.cpp` 的 `run()` 函数中（`setupStep()` 调用**之后**）添加映射逻辑。
*   **映射关系 (符合 MIT 习惯)**：
    *   **右摇杆**：控制 Roll (左右倾斜) / Pitch (前后倾斜)。
    *   **左摇杆**：控制 Yaw (转向，需取反 `-LX`) / Height (高度升降)。
*   **灵敏度**：设置缩放系数 `kScaleRPY = 0.35`, `kScaleH = 0.4`，手感柔和。
*   **控制模式**：`FSM_State_BalanceStand.cpp` 修改为**松手自动回中**（比例控制），松开摇杆后机器人自动恢复水平和初始高度。

### 4.6 遗留风险提示
*   **Locomotion (原地踏步)**：**极高风险！**
    *   目前的接触估计器 (`ContactEstimator`) 是基于步态计划的盲信逻辑，不依赖真实触地反馈。
    *   **未验证项**：IMU 的线性加速度 (Acc) 方向尚未经过严格验证（虽然 RPY 已修）。如果 Acc 方向也是反的，卡尔曼滤波会瞬间发散导致炸机。
    *   **建议**：在严格验证 Acc 方向（前推 AccX 为正，静止 Z 为 +9.8）之前，**严禁尝试 Trot 步态**。

---

## 常用 Git 命令速查

### 本地操作
*   **补充提交**（修改上一次 commit）：
    ```bash
    git commit --amend --no-edit
    ```
*   **强制推送**（覆盖远端）：
    ```bash
    git push --force-with-lease
    ```

### 远程同步
*   **强制重置到远端 main**（丢弃本地所有修改）：
    ```bash
    git fetch origin && git reset --hard origin/main && git clean -fd
    ```
    *   `git fetch origin`: 获取最新元数据
    *   `git reset --hard origin/main`: 强制移动指针并重置工作区
    *   `git clean -fd`: 删除未跟踪的文件和目录