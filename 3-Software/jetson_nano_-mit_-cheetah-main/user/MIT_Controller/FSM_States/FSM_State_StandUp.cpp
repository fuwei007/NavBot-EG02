/*============================= Stand Up ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "FSM_State_StandUp.h"
#include <algorithm>  // std::min / max
#include <cstdio>
#include "Utilities/Log.h"

/**
 * Constructor
 */
template <typename T>
FSM_State_StandUp<T>::FSM_State_StandUp(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::STAND_UP, "STAND_UP"),
      _ini_foot_pos(4) {
  // pre control safety
  this->checkSafeOrientation = false;

  // post control safety
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;
}

template <typename T>
void FSM_State_StandUp<T>::onEnter() {
  this->nextStateName = this->stateName;
  this->transitionData.zero();
  iter = 0;

  // 记录趴下时的足端初始位置
  for (size_t leg(0); leg < 4; ++leg) {
    _ini_foot_pos[leg] = this->_data->_legController->datas[leg].p;
  }
}

/**
 * 每个控制周期调用
 */
template <typename T>
void FSM_State_StandUp<T>::run() {
  // 只对 MINI_CHEETAH 生效
  if (this->_data->_quadruped->_robotType != RobotType::MINI_CHEETAH) {
    return;
  }

  // 站立目标高度（正数），可以以后做成 userParameters
  // 调高到 0.33 以解决实际高度过低导致小腿贴地的问题
  const T hStandDefault = 0.33; // m
  const T hStand = hStandDefault;

  // 机器人质量 & 重力
  const T massDefault = 9.0;
  const T mass = massDefault;
  const T g = 9.81;

  // 控制周期
  const T dt = this->_data->controlParameters->controller_dt;

  // 站起总时间（秒），别太快，1.0~2.0 比较温和
  const T standTime = 1.2;

  // 0~1 的归一化进度
  T progress = iter * dt / standTime;
  if (progress > T(1)) progress = T(1);

  // smoothstep：比线性更平滑（前后速度为 0）
  T s = progress * progress * (T(3) - T(2) * progress);

  // 足端偏移：从 userParameters 读取
  const T x_raw[4] = {
      (T)this->_data->userParameters->foot_x_offset_0,
      (T)this->_data->userParameters->foot_x_offset_1,
      (T)this->_data->userParameters->foot_x_offset_2,
      (T)this->_data->userParameters->foot_x_offset_3};

  const T y_raw[4] = {
      (T)this->_data->userParameters->foot_y_offset_0,
      (T)this->_data->userParameters->foot_y_offset_1,
      (T)this->_data->userParameters->foot_y_offset_2,
      (T)this->_data->userParameters->foot_y_offset_3};

  const T z_raw[4] = {
      (T)this->_data->userParameters->foot_z_offset_0,
      (T)this->_data->userParameters->foot_z_offset_1,
      (T)this->_data->userParameters->foot_z_offset_2,
      (T)this->_data->userParameters->foot_z_offset_3};

  // 对 offset 做一点限幅，防止一次拉太离谱
  const T maxX = (T)0.05;   // 5 cm
  const T maxY = (T)0.15;   // 15 cm（你的 rear 0.10 在这个范围内）
  const T maxZ = (T)0.05;   // 5 cm 做微调够用了

  auto clamp = [](T v, T lo, T hi) {
    return std::max(lo, std::min(hi, v));
  };

  // 每条腿
  for (int leg = 0; leg < 4; ++leg) {
    auto& cmd = this->_data->_legController->commands[leg];

    // 只用笛卡尔空间 PD，关节空间清零，避免“抢控制权”
    cmd.kpJoint.setZero();
    cmd.kdJoint.setZero();
    cmd.qDes.setZero();
    cmd.qdDes.setZero();

    // 稍微比你原来 500 软一点，减小扭矩尖峰
    cmd.kpCartesian = Vec3<T>(350, 350, 450).asDiagonal();
    cmd.kdCartesian = Vec3<T>(  6,   6,   8).asDiagonal();

    // 基于初始足端位置 + offset 生成目标
    Vec3<T> pDes = _ini_foot_pos[leg];

    const T xOff = clamp(x_raw[leg], -maxX, maxX);
    const T yOff = clamp(y_raw[leg], -maxY, maxY);
    const T zOff = clamp(z_raw[leg], -maxZ, maxZ);

    // 1. X: 仍旧是初始位置 + 微调 offset
    pDes[0] += xOff;

    // 2. Y: 自动收腿逻辑
    // 计算标准站立时的 Y 坐标 (Hip位置 + AbadLink长度)
    Vec3<T> pHip = this->_data->_quadruped->getHipLocation(leg);
    T sideSign = this->_data->_quadruped->getSideSign(leg);
    T l_abad = this->_data->_quadruped->_abadLinkLength;
    T yStand = pHip[1] + sideSign * l_abad;

    // 从初始趴下的 Y 平滑过渡到标准站立 Y，并保留 offset 微调能力
    pDes[1] = _ini_foot_pos[leg][1] + s * (yStand - _ini_foot_pos[leg][1]) + yOff;

    // 3. Z: 从趴下高度平滑插到 -hStand，再加上单腿微调 zOff
    const T zTarget = -hStand + zOff;
    pDes[2] = _ini_foot_pos[leg][2] + s * (zTarget - _ini_foot_pos[leg][2]);

    cmd.pDes = pDes;
    cmd.vDes.setZero(); // 静态站立，速度期望置零

    // 重力补偿前馈：随 s 渐增
    const T fz = (mass * g / 4) * s;
    cmd.forceFeedForward = Vec3<T>(0, 0, fz);
  }

  // 打 log 方便你调
  static int s_standup_log_decimator = 0;
  if ((s_standup_log_decimator++ % 500) == 0) {
    for (int leg = 0; leg < 4; ++leg) {
      auto& cmd = this->_data->_legController->commands[leg];
      LOG_INFO("[STAND] leg{} prog={:.2f} pDes=[{:.3f} {:.3f} {:.3f}] fz={:.2f}",
          leg,
          (double)progress,
          (double)cmd.pDes[0],
          (double)cmd.pDes[1],
          (double)cmd.pDes[2],
          (double)cmd.forceFeedForward[2]);
    }
  }
}

/*** 下面 checkTransition / transition / onExit 和你原来一致 ***/

template <typename T>
FSM_StateName FSM_State_StandUp<T>::checkTransition() {
  this->nextStateName = this->stateName;
  iter++;

  switch ((int)this->_data->controlParameters->control_mode) {
    case K_STAND_UP:
      break;
    case K_BALANCE_STAND:
      this->nextStateName = FSM_StateName::BALANCE_STAND;
      break;
    case K_LOCOMOTION:
      this->nextStateName = FSM_StateName::LOCOMOTION;
      break;
    case K_VISION:
      this->nextStateName = FSM_StateName::VISION;
      break;
    case K_PASSIVE:
      this->nextStateName = FSM_StateName::PASSIVE;
      break;
    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << K_PASSIVE << " to "
                << this->_data->controlParameters->control_mode << std::endl;
  }
  return this->nextStateName;
}

template <typename T>
TransitionData<T> FSM_State_StandUp<T>::transition() {
  switch (this->nextStateName) {
    case FSM_StateName::PASSIVE:
    case FSM_StateName::BALANCE_STAND:
    case FSM_StateName::LOCOMOTION:
    case FSM_StateName::VISION:
      this->transitionData.done = true;
      break;
    default:
      std::cout << "[CONTROL FSM] Something went wrong in transition"
                << std::endl;
  }
  return this->transitionData;
}

template <typename T>
void FSM_State_StandUp<T>::onExit() {
  // Nothing
}

template class FSM_State_StandUp<float>;
