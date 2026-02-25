# MIT Mini Cheetah SPIne 程序摘要

## 系统角色
- 负责连接上位机与腿部驱动器，通过 SPI 接收控制指令并经双路 CAN 下发命令。
- 使用 `spi_command_t` 和 `spi_data_t` 以 16 bit 对齐的缓存同步两条腿、各三个关节的指令与反馈。
- 借助 `math_ops` 提供的量化函数限制位置、速度、增益与力矩在安全范围内。

## SPI 从机链路
- `SPISlave` 以 16 bit 帧、5 MHz 频率工作，`cs.fall()` 触发 `spi_isr` 处理一次完整收发。
- 会话开始时写入 `tx_buff` 首个半字，片选拉低期间同步收发 66 个 16 bit 数据单元。
- `xor_checksum()` 对 128 字节的主控指令做校验，失败时将 `spi_data.flags[1]` 置为 `0xdead` 反馈异常。
- `control()` 根据最新 `spi_command` 生成控制量，再将 `spi_data` 重新打包进 SPI 发送缓冲。

## CAN 电机协议
- 总线参数：双路 1 Mbps 标准帧 CAN；驱动 ID 依次为 0x1（转髋）、0x2（髋）、0x3（膝），左右腿各占一条总线。
- 下行 8 字节格式：Byte0-1 位置、Byte2-3 速度、Byte3-4 比例增益、Byte5-6 微分增益、Byte6-7 前馈力矩；所有字段经 `float_to_uint` 量化为 16/12 bit。
- 模式帧使用全 0xFF 数据，末字节 `0xFC`/`0xFD`/`0xFE` 分别表示进入电机模式、退出电机模式、执行零位。
- 反馈帧 6 字节：Byte0 为关节 ID，Byte1-2 位置，Byte3-4 携带速度（12 bit）与电流高 4 bit，Byte5 为电流低 8 bit，`unpack_reply()` 负责解析并写入 `leg_state`。

## 控制与安全逻辑
- `control()` 读取 `spi_command.flags[0]` 的最低位决定是否发送 `EnterMotorMode()` 或 `ExitMotorMode()`，并维护 `enabled` 状态。
- `estop` 急停输入被拉低时清空两条腿控制量、点亮指示灯，并在 `spi_data.flags` 中返回 `0xdead` 告知上位机。
- `softstop_joint()` 在关节接近软限位时关闭位置环，仅保留阻尼并记录软限位事件，将状态反馈至 `flags`。

## 调试与辅助功能
- 串口调试：`pc` 设定为 921600 bps，`serial_isr()` 接收键盘命令，其中 `m` 进入电机模式、`Esc` 退出、`z` 零位、`s` 触发站立逻辑（入口保留，具体动作可自行扩展）。
- 数据观测：`sendCMD()` 可挂接到 `Ticker` 周期打印左右腿三关节的位置数据，便于验证传感器与控制闭环响应。
- 运行日志：在 SPI 初始化、SPI 中断及电机启停路径中使用 `pc.printf()` 输出关键事件，帮助定位通信或安全状态问题。
- SPI 配置：`init_spi()` 负责从机格式与频率设置，并将 `cs.fall()` 绑定到中断，保证上位机拉低片选即可触发处理。
- 主循环：持续轮询两路 CAN，将最新反馈写入 `l*_state`，确保下一次 SPI 事务能够返回最新关节状态。
