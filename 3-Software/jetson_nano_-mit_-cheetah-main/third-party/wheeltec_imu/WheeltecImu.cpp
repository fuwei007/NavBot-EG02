#ifdef linux

#include "WheeltecImu.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdio>

#include "Utilities/Log.h"
#include "Math/orientation_tools.h"

// ========================= 可配置宏 =========================
// 若设备发送四元数顺序为 x,y,z,w，请设为 1；默认认为是 w,x,y,z
#ifndef WHEELTEC_QUAT_IS_XYZW
#define WHEELTEC_QUAT_IS_XYZW 0
#endif

// 设备若输出 earth->body（常见 e2b），而上层需要 body->earth（b2e），置 1 以做共轭
#ifndef WHEELTEC_QUAT_IS_EARTH_TO_BODY
#define WHEELTEC_QUAT_IS_EARTH_TO_BODY 1
#endif

// 是否启用安装角补偿（右乘到机体系）：q_final = q_raw * q_install
#ifndef WHEELTEC_USE_INSTALL_COMPENSATION
#define WHEELTEC_USE_INSTALL_COMPENSATION 0
#endif

// 安装角补偿（单位：度）
#ifndef WHEELTEC_INSTALL_RPY_DEG_ROLL
#define WHEELTEC_INSTALL_RPY_DEG_ROLL  0.0f
#endif
#ifndef WHEELTEC_INSTALL_RPY_DEG_PITCH
#define WHEELTEC_INSTALL_RPY_DEG_PITCH 0.0f
#endif
#ifndef WHEELTEC_INSTALL_RPY_DEG_YAW
#define WHEELTEC_INSTALL_RPY_DEG_YAW   0.0f
#endif

// 启动约 1s 后打印一次 RPY/acc，便于自检
#ifndef WHEELTEC_DEBUG_PRINT
#define WHEELTEC_DEBUG_PRINT 1
#endif
// ==========================================================

namespace {
constexpr uint8_t kHeader0 = 0x59;
constexpr uint8_t kHeader1 = 0x53;
constexpr size_t  kMinPacketSize = 7;
constexpr float   kScale    = 1e-6f;
constexpr float   kDegToRad = static_cast<float>(M_PI / 180.0);

constexpr uint8_t kAccelId      = 0x10;
constexpr uint8_t kGyroId       = 0x20;
constexpr uint8_t kQuaternionId = 0x41;

inline int32_t readInt32(const uint8_t* ptr) {
  return static_cast<int32_t>(ptr[0]) |
        (static_cast<int32_t>(ptr[1]) << 8) |
        (static_cast<int32_t>(ptr[2]) << 16) |
        (static_cast<int32_t>(ptr[3]) << 24);
}

inline speed_t baudToSpeed(int baud) {
  switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
#ifdef B230400
    case 230400: return B230400;
#endif
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
    default:     return B460800;
  }
}

// 本地实现：wxyz 的四元数共轭
inline Vec4<float> quatConjugateLocal(const Vec4<float>& q_wxyz) {
  Vec4<float> out;
  out << q_wxyz[0], -q_wxyz[1], -q_wxyz[2], -q_wxyz[3];
  return out;
}

} // namespace

// ------- 调试计时（打印一次） -------
static std::chrono::steady_clock::time_point g_t0 = std::chrono::steady_clock::now();

WheeltecImu::WheeltecImu() {
  _rxBuffer.reserve(512);
  // 四元数内部约定为 wxyz
  _quat << 1.f, 0.f, 0.f, 0.f;

  // 依据手册：设备已是 ENU/FLU，默认不再做 NED->ENU 之类旋转
  _vectorRotation.setIdentity();

  // 默认不改世界参考（左乘恒等）
  _leftQuat  << 1.f, 0.f, 0.f, 0.f;

#if WHEELTEC_USE_INSTALL_COMPENSATION
  {
    const float r = WHEELTEC_INSTALL_RPY_DEG_ROLL  * (float)M_PI / 180.f;
    const float p = WHEELTEC_INSTALL_RPY_DEG_PITCH * (float)M_PI / 180.f;
    const float y = WHEELTEC_INSTALL_RPY_DEG_YAW   * (float)M_PI / 180.f;
    const Vec3<float> rpy(r, p, y);
    _rightQuat = ori::rpyToQuat(rpy);   // q_final = q_raw * q_install
  }
#else
  _rightQuat << 1.f, 0.f, 0.f, 0.f;
#endif
}

WheeltecImu::~WheeltecImu() {
  closePort();
}

void WheeltecImu::closePort() {
  if (_fd >= 0) {
    close(_fd);
    _fd = -1;
  }
  _initialized = false;
}

bool WheeltecImu::configurePort(int baud_rate) {
  if (_fd < 0) return false;

  termios tty;
  if (tcgetattr(_fd, &tty) != 0) {
    LOG_ERROR("[WheeltecImu] tcgetattr: {}", std::strerror(errno));
    return false;
  }

  cfmakeraw(&tty);
  const speed_t speed = baudToSpeed(baud_rate);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(_fd, TCSANOW, &tty) != 0) {
    LOG_ERROR("[WheeltecImu] tcsetattr: {}", std::strerror(errno));
    return false;
  }

  tcflush(_fd, TCIOFLUSH);
  return true;
}

bool WheeltecImu::tryInit(const std::string& device_path, int baud_rate) {
  closePort();

  _fd = open(device_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (_fd < 0) {
    LOG_ERROR("[WheeltecImu] open: {}", std::strerror(errno));
    return false;
  }

  if (!configurePort(baud_rate)) {
    closePort();
    return false;
  }

  _rxBuffer.clear();
  _goodPackets = 0;
  _badPackets  = 0;
  _initialized = true;

  LOG_INFO("[WheeltecImu] Connected to {} @ {} baud",
         device_path, baud_rate);
  return true;
}

void WheeltecImu::run() {
  if (_fd < 0) {
    usleep(100000);
    return;
  }

  uint8_t temp[256];
  const ssize_t bytes = read(_fd, temp, sizeof(temp));

  if (bytes > 0) {
    _rxBuffer.insert(_rxBuffer.end(), temp, temp + bytes);
    processBuffer();
  } else if (bytes < 0) {
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      LOG_ERROR("[WheeltecImu] read: {}", std::strerror(errno));
      ++_badPackets;
      usleep(1000);
    }
  } else {
    usleep(1000);
  }
}

void WheeltecImu::processBuffer() {
  while (_rxBuffer.size() >= kMinPacketSize) {
    if (_rxBuffer[0] != kHeader0 || _rxBuffer[1] != kHeader1) {
      _rxBuffer.erase(_rxBuffer.begin());
      continue;
    }

    const uint8_t payload_len = _rxBuffer[4];
    if (payload_len > 200) { // 简单容错
      _rxBuffer.erase(_rxBuffer.begin());
      ++_badPackets;
      continue;
    }

    const size_t payload_end = 5 + payload_len;
    const size_t packet_len  = payload_len + kMinPacketSize;
    if (_rxBuffer.size() < packet_len) break;

    // 校验
    uint8_t ck1 = 0, ck2 = 0;
    for (size_t i = 2; i < payload_end; ++i) {
      ck1 += _rxBuffer[i];
      ck2 += ck1;
    }
    const uint8_t ref_ck1 = _rxBuffer[payload_end];
    const uint8_t ref_ck2 = _rxBuffer[payload_end + 1];
    if (ck1 != ref_ck1 || ck2 != ref_ck2) {
      _rxBuffer.erase(_rxBuffer.begin());
      ++_badPackets;
      continue;
    }

    Packet packet;
    size_t idx = 5;

    while (idx + 2 <= payload_end) {
      const uint8_t data_id  = _rxBuffer[idx++];
      const uint8_t data_len = _rxBuffer[idx++];

      if (idx + data_len > payload_end) {
        idx = payload_end;
        break;
      }

      const uint8_t* data_ptr = &_rxBuffer[idx];

      switch (data_id) {
        case kAccelId:
          if (data_len == 12) {
            packet.hasAccel = true;
            packet.accel <<
              static_cast<float>(readInt32(data_ptr))      * kScale,
              static_cast<float>(readInt32(data_ptr + 4))  * kScale,
              static_cast<float>(readInt32(data_ptr + 8))  * kScale;
          }
          break;
        case kGyroId:
          if (data_len == 12) {
            packet.hasGyro = true;
            packet.gyro <<
              static_cast<float>(readInt32(data_ptr))      * kScale,
              static_cast<float>(readInt32(data_ptr + 4))  * kScale,
              static_cast<float>(readInt32(data_ptr + 8))  * kScale;
            packet.gyro *= kDegToRad; // 度->弧度
          }
          break;
        case kQuaternionId:
          if (data_len == 16) {
            packet.hasQuat = true;
#if WHEELTEC_QUAT_IS_XYZW
            // 设备若发 xyzw，这里重排成 wxyz
            Vec4<float> q_xyzw;
            q_xyzw <<
              static_cast<float>(readInt32(data_ptr))      * kScale,
              static_cast<float>(readInt32(data_ptr + 4))  * kScale,
              static_cast<float>(readInt32(data_ptr + 8))  * kScale,
              static_cast<float>(readInt32(data_ptr + 12)) * kScale;
            packet.quat << q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2];
#else
            // 默认设备发 wxyz
            packet.quat <<
              static_cast<float>(readInt32(data_ptr))      * kScale,
              static_cast<float>(readInt32(data_ptr + 4))  * kScale,
              static_cast<float>(readInt32(data_ptr + 8))  * kScale,
              static_cast<float>(readInt32(data_ptr + 12)) * kScale;
#endif
          }
          break;
        default:
          break;
      }

      idx += data_len;
    }

    if (packet.hasAccel || packet.hasGyro || packet.hasQuat) {
      handlePacket(packet);
      ++_goodPackets;
    } else {
      ++_badPackets;
    }

    _rxBuffer.erase(_rxBuffer.begin(), _rxBuffer.begin() + packet_len);
  }
}

void WheeltecImu::handlePacket(const Packet& packet) {
  std::lock_guard<std::mutex> lock(_dataMutex);

  if (packet.hasAccel) {
    _accel = _vectorRotation * packet.accel; // 默认恒等
    _accel[0] = -_accel[0]; // 反转 X 轴加速度
    _accel[1] = -_accel[1]; // 反转 Y 轴加速度
  }

  if (packet.hasGyro) {
    _gyro = _vectorRotation * packet.gyro;   // 默认恒等
  }

  if (packet.hasQuat) {
    Vec4<float> q = packet.quat;   // wxyz

    // 单位化
    float n = q.norm();
    if (n > 1e-6f) q /= n;
    else           q << 1.f, 0.f, 0.f, 0.f;

    // 若设备给的是 earth->body（e2b），而上层需要 body->earth（b2e）：取共轭
#if WHEELTEC_QUAT_IS_EARTH_TO_BODY
    q = quatConjugateLocal(q);  // b2e = conj(e2b)
#endif

    // q_final = q_left * ( q * q_right )  （左乘世界修正；右乘安装补偿到机体系）
    Vec4<float> transformed = ori::quatProduct(_leftQuat, ori::quatProduct(q, _rightQuat));
    n = transformed.norm();
    if (n > 1e-6f) transformed /= n;
    else           transformed << 1.f, 0.f, 0.f, 0.f;

    // [Yaw Axis Correction]
    // Convert to RPY, invert Yaw, convert back to Quat.
    // This fixes the inverted yaw issue observed in tests.
    Vec3<float> rpy = ori::quatToRPY(transformed);
    rpy[0] = -rpy[0]; // Invert Roll
    rpy[1] = -rpy[1]; // Invert Pitch
    rpy[2] = -rpy[2]; // Invert Yaw
    transformed = ori::rpyToQuat(rpy);

    _quat = transformed;
  }
}

void WheeltecImu::copyToVectorNav(VectorNavData* data) const {
  if (!data) return;
  std::lock_guard<std::mutex> lock(_dataMutex);
  data->accelerometer = _accel;
  data->gyro          = _gyro;
  // VectorNavData expects quaternion ordered as [x y z w] (see VectorNavOrientationEstimator)
  data->quat << _quat[1], _quat[2], _quat[3], _quat[0];
}

void WheeltecImu::updateLCM(microstrain_lcmt* message) const {
  if (!message) return;
  std::lock_guard<std::mutex> lock(_dataMutex);

  for (int i = 0; i < 4; ++i) message->quat[i] = _quat[i];

  const Vec3<float> rpy = ori::quatToRPY(_quat);
  for (int i = 0; i < 3; ++i) {
    message->rpy[i]   = rpy[i];
    message->acc[i]   = _accel[i];
    message->omega[i] = _gyro[i];
  }

  message->good_packets = _goodPackets.load();
  message->bad_packets  = _badPackets.load();

#if WHEELTEC_DEBUG_PRINT
  if (!_printedOnce) {
    auto dt = std::chrono::steady_clock::now() - g_t0;
    if (std::chrono::duration_cast<std::chrono::milliseconds>(dt).count() > 1000) {
      LOG_INFO("[WheeltecImu][DEBUG] RPY(deg)= [{:.2f}, {:.2f}, {:.2f}], acc= [{:.3f}, {:.3f}, {:.3f}]",
             message->rpy[0] * 180.0/M_PI, message->rpy[1] * 180.0/M_PI, message->rpy[2] * 180.0/M_PI,
             message->acc[0], message->acc[1], message->acc[2]);
      _printedOnce = true; // 现在是 mutable，合法
    }
  }
#endif
}

#endif  // linux
