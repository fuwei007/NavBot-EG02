/*! @file MiniCheetah.h
 *  @brief Utility function to build a Mini Cheetah Quadruped object
 *
 * This file is based on MiniCheetahFullRotorModel_mex.m and builds a model
 * of the Mini Cheetah robot.  The inertia parameters of all bodies are
 * determined from CAD.
 *
 */

#ifndef PROJECT_MINICHEETAH_H
#define PROJECT_MINICHEETAH_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"

/*!
 * Generate a Quadruped model of Mini Cheetah
 */
template <typename T>
Quadruped<T> buildMiniCheetah() {
  Quadruped<T> cheetah;
  cheetah._robotType = RobotType::MINI_CHEETAH;

  // ----------------- 机身几何与质量（按你实测） -----------------
  cheetah._bodyMass   = 3.3f;    // 暂时沿用 MIT 原版机身质量（只算机身，不含腿）
  cheetah._bodyLength = 0.50f;   // 机身长约 50 cm
  cheetah._bodyWidth  = 0.34f;   // 机身宽约 34 cm
  cheetah._bodyHeight = 0.115f;  // 机身厚度约 11.5 cm

  // 齿比：ABAD/hip 为 6 : 1；knee 为 行星 6 : 1 + 同步带 30/19
  cheetah._abadGearRatio = 6.f;
  cheetah._hipGearRatio  = 6.f;
  cheetah._kneeGearRatio = 6.f * 30.f / 19.f;  // ≈ 9.47

  // ----------------- 腿部几何（按你实测） -----------------
  cheetah._abadLinkLength   = 0.07f;   // ABAD 到 hip 关节的侧向距离 ≈ 7 cm
  cheetah._hipLinkLength    = 0.22f;   // 大腿长度 ≈ 22 cm
  cheetah._kneeLinkLength   = 0.22f;   // 小腿长度 ≈ 22 cm
  cheetah._kneeLinkY_offset = 0.f;     // 你量的基本为 0
  cheetah._maxLegLength     = cheetah._hipLinkLength + cheetah._kneeLinkLength; // ≈ 0.30 m

  // ----------------- 电机与关节参数（按 M24） -----------------
  // 说明：M24 数据表里的 6 N·m / 23.5 N·m 是输出轴扭矩（带减速后），这里先沿用原版 3 N·m，
  // 防止控制太猛；后面可以根据实测再放开。
  cheetah._motorTauMax = 4.f;          // 先保持和 MIT 相同的安全扭矩限制
  cheetah._batteryV    = 24.f;

  // 扭矩常数：数据表是 1.18 N·m/A（输出端），这里折算回电机侧再乘极对数比较复杂。
  // 先保留原来的 0.05，用起来比较温和，日后可以根据实测再调。
  cheetah._motorKT = 0.196f;
  cheetah._motorR  = 0.173f;           // 保持原版电机内阻
  cheetah._jointDamping      = 0.01f;
  cheetah._jointDryFriction  = 0.2f;

  // ----------------- 转子转动惯量（沿用 MIT 原版） -----------------
  Mat3<T> rotorRotationalInertiaZ;
  rotorRotationalInertiaZ << 33, 0, 0,
                             0, 33, 0,
                             0, 0, 63;
  rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

  Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
  Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
  Mat3<T> rotorRotationalInertiaX =
      RY * rotorRotationalInertiaZ * RY.transpose();
  Mat3<T> rotorRotationalInertiaY =
      RX * rotorRotationalInertiaZ * RX.transpose();

  // 粗略放大腿部惯量，防止 WBC 低估腿部动力学
  T inertia_scale = 1.0; // 回调到 1.0，测试是否惯量放大过大导致不稳定

  Mat3<T> abadRotationalInertia;
  abadRotationalInertia << 381, 58, 0.45,
                           58, 560, 0.95,
                           0.45, 0.95, 444;
  abadRotationalInertia = abadRotationalInertia * 1e-6 * inertia_scale; // <--- 放大

  Mat3<T> hipRotationalInertia;
  hipRotationalInertia << 1983, 245, 13,
                          245, 2103, 1.5,
                          13, 1.5, 408;
  hipRotationalInertia = hipRotationalInertia * 1e-6 * inertia_scale; // <--- 放大

  Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
  kneeRotationalInertiaRotated << 6, 0, 0,
                                  0, 248, 0,
                                  0, 0, 245;
  kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6 * inertia_scale; // <--- 放大
  kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
  Vec3<T> kneeCOM(0, 0, -0.061);
  SpatialInertia<T> kneeInertia(0.064 * inertia_scale, kneeCOM, kneeRotationalInertia); // 质量也相应放大

  Vec3<T> abadCOM(0, 0.036, 0);  // LEFT
  SpatialInertia<T> abadInertia(0.54 * inertia_scale, abadCOM, abadRotationalInertia);

  Vec3<T> hipCOM(0, 0.016, -0.02);
  SpatialInertia<T> hipInertia(0.634 * inertia_scale, hipCOM, hipRotationalInertia);

  Vec3<T> rotorCOM(0, 0, 0);
  SpatialInertia<T> rotorInertiaX(0.055, rotorCOM, rotorRotationalInertiaX);
  SpatialInertia<T> rotorInertiaY(0.055, rotorCOM, rotorRotationalInertiaY);

  Mat3<T> bodyRotationalInertia;
  // 将之前过度放大的机身惯量回调，以 x1.5 作为保守估算
  bodyRotationalInertia << 11253 * 1.5, 0, 0, // 原值 11253
                           0, 36203 * 1.5, 0, // 原值 36203
                           0, 0, 42673 * 1.5; // 原值 42673
  bodyRotationalInertia = bodyRotationalInertia * 1e-6;
  
  // CoM 微调：-0.005 (从 -0.02 回调，防止向后坐倒)
  Vec3<T> bodyCOM(-0.005, 0, 0);
  SpatialInertia<T> bodyInertia(cheetah._bodyMass, bodyCOM,
                                bodyRotationalInertia);

  cheetah._abadInertia      = abadInertia;
  cheetah._hipInertia       = hipInertia;
  cheetah._kneeInertia      = kneeInertia;
  cheetah._abadRotorInertia = rotorInertiaX;
  cheetah._hipRotorInertia  = rotorInertiaY;
  cheetah._kneeRotorInertia = rotorInertiaY;
  cheetah._bodyInertia      = bodyInertia;

  // ----------------- 关节在机身上的安装位置（按你测的 ABAD 偏移） -----------------
  // 你量的是前右腿：x = 0.20 m (L=40cm), y = 0.065 m (W=13cm)，相对机身几何中心
  // 这里存的是“正方向偏移量”，具体前/后、左/右的正负号在别处根据腿编号加。
  cheetah._abadLocation      = Vec3<T>(0.20f, 0.065f, 0.f);
  cheetah._abadRotorLocation = Vec3<T>(0.20f, 0.065f, 0.f);

  cheetah._hipLocation       = Vec3<T>(0, cheetah._abadLinkLength, 0);
  cheetah._hipRotorLocation  = Vec3<T>(0, 0.04, 0);
  cheetah._kneeLocation      = Vec3<T>(0, 0, -cheetah._hipLinkLength);
  cheetah._kneeRotorLocation = Vec3<T>(0, 0, 0);

  return cheetah;
}

#endif  // PROJECT_MINICHEETAH_H
