#pragma once

#ifdef linux

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "SimUtilities/IMUTypes.h"
#include "cppTypes.h"
#include "microstrain_lcmt.hpp"

/**
 * Wheeltec H30 IMU 简易驱动（与 Mini Cheetah 上层接口兼容）。
 *
 * 约定：
 * - 设备原生为 ENU/FLU（世界 ENU，载体 FLU），不做 NED->ENU 旋转；
 * - 向量与四元数使用同一坐标约定；
 * - 可选：若设备给的是 earth->body（e2b），转为 body->earth（b2e）；
 * - 可选：若设备给四元数为 xyzw，重排为 wxyz；
 * - 可选：安装角补偿（右乘到机体系）。
 */
class WheeltecImu {
 public:
  WheeltecImu();
  ~WheeltecImu();

  WheeltecImu(const WheeltecImu&) = delete;
  WheeltecImu& operator=(const WheeltecImu&) = delete;

  /** 初始化串口：device_path 如 "/dev/ttyACM0"，baud_rate 如 460800 */
  bool tryInit(const std::string& device_path, int baud_rate);

  /** 周期调用：拉取串口数据并解析 */
  void run();

  /** 线程安全地拷贝数据给上层 VectorNavData（保持项目原型） */
  void copyToVectorNav(VectorNavData* data) const;

  /** 写入 LCM 消息（便于可视化/日志） */
  void updateLCM(microstrain_lcmt* message) const;

  /** 串口是否已打开 */
  bool isInitialized() const { return _initialized; }

 private:
  struct Packet {
    bool hasAccel{false};
    bool hasGyro{false};
    bool hasQuat{false};
    Vec3<float> accel{Vec3<float>::Zero()};
    Vec3<float> gyro{Vec3<float>::Zero()};
    Vec4<float> quat{Vec4<float>::Zero()}; // 统一 wxyz
  };

  void closePort();
  bool configurePort(int baud_rate);
  void processBuffer();
  void handlePacket(const Packet& packet);

  // 串口
  int _fd{-1};
  std::vector<uint8_t> _rxBuffer;

  // 数据（线程保护）
  mutable std::mutex _dataMutex;
  Vec3<float> _accel{Vec3<float>::Zero()};
  Vec3<float> _gyro{Vec3<float>::Zero()};
  Vec4<float> _quat{Vec4<float>::Zero()};   // wxyz

  // 姿态/向量修正
  Vec4<float> _leftQuat;       // 世界参考修正（通常恒等）
  Vec4<float> _rightQuat;      // 安装角补偿（右乘到机体系）
  Mat3<float> _vectorRotation; // 对 acc/gyro 的线性旋转（通常恒等）

  // 状态
  std::atomic<bool> _initialized{false};
  mutable std::atomic<int64_t> _goodPackets{0};
  mutable std::atomic<int64_t> _badPackets{0};

  // 调试（updateLCM 是 const，需要 mutable）
  mutable bool _printedOnce{false};
};

#endif  // linux
