// rt_spi_jetson.cpp — Jetson Nano spidev 适配（Board0=/dev/spidev0.0, Board1=/dev/spidev0.1）
// SPI: Mode0, MSB-first, 缺省 8 bits/word, 1 MHz, 每帧 CS 释放延时 2us
// 帧: 66×16bit；仅前 30×16bit 为有效数据（12 个 float + flags + 32-bit 校验）
//
// 需要你项目内的 rt/rt_spi.h 提供：
//   spi_command_t, spi_data_t, spine_cmd_t, spi_torque_t, K_EXPECTED_COMMAND_SIZE, K_EXPECTED_DATA_SIZE 等
//
// 可在编译期覆盖的缺省：
//   -DSPI_DEV0_PATH=\"/dev/spidev0.0\" -DSPI_DEV1_PATH=\"/dev/spidev0.1\"
//   -DMINC_SPI_HZ=1000000u -DWANT_BPW=8 -DSPI_FRAME_DELAY_US=2 -DSPI_DEBUG=1 -DSPI_DUMP_EVERY=50
//   -DK_WORDS_PER_MESSAGE=66 -DK_SPI_DATA_WORDS=30 -DK_KNEE_OFFSET_POS=0.f

#ifdef __linux__

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cerrno>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cmath>

#include <pthread.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include "Utilities/Log.h"
#include "rt/rt_spi.h"  // 由你的工程提供

#ifndef SPI_DEV0_PATH
#define SPI_DEV0_PATH "/dev/spidev0.0"
#endif
#ifndef SPI_DEV1_PATH
#define SPI_DEV1_PATH "/dev/spidev0.1"
#endif
#ifndef MINC_SPI_HZ
#define MINC_SPI_HZ 50000u
#endif
#ifndef WANT_BPW
#define WANT_BPW 8
#endif
#ifndef SPI_FRAME_DELAY_US
#define SPI_FRAME_DELAY_US 2
#endif
#ifndef SPI_DEBUG
#define SPI_DEBUG 0
#endif
#ifndef SPI_DUMP_EVERY
#define SPI_DUMP_EVERY 50
#endif
#ifndef K_WORDS_PER_MESSAGE
#define K_WORDS_PER_MESSAGE 66
#endif
#ifndef K_SPI_DATA_WORDS
#define K_SPI_DATA_WORDS 30
#endif
#ifndef K_KNEE_OFFSET_POS
#define K_KNEE_OFFSET_POS 0.f
#endif

#if SPI_DEBUG
  #define DBG(...)  std::printf(__VA_ARGS__)
#else
  #define DBG(...)  do{}while(0)
#endif

static int      g_fd[2]       = {-1,-1};
static uint8_t  g_real_bpw[2] = {8,8};
static uint8_t  g_mode        = SPI_MODE_0;
static uint8_t  g_lsb_first   = 0;        // MSB-first

static pthread_mutex_t g_spi_mutex;

spi_command_t   spi_command_drv;
spi_data_t      spi_data_drv;
spi_torque_t    spi_torque;

static spine_cmd_t  g_spine_cmd;
static int          g_half_order[2] = {-1,-1};  // -1 未定；0=lohi(hi<<16|lo)，1=hilo(lo<<16|hi)
#if SPI_DEBUG
static uint64_t     g_frame_cnt [2] = {0,0};
#endif
static int          g_checksum_failures[2] = {0,0};
static int          g_last_good_iter[2] = {0,0};
static bool         g_warned_bad_spi_cmd = false;

// 机械侧参数（与你之前版一致）
static const float max_torque[3]      = {17.f,17.f,26.f};
static const float wimp_torque[3]     = { 6.f, 6.f, 6.f};
static const float disabled_torque[3] = { 0.f, 0.f, 0.f};


// only used for actual robot
// const float abad_side_sign[4] = {-1.f, -1.f, 1.f, 1.f};
// const float hip_side_sign[4] = {-1.f, 1.f, -1.f, 1.f};
// const float knee_side_sign[4] = {-.6429f, .6429f, -.6429f, .6429f};

// FR, FL, RR, RL
static const float abad_side_sign[4] = {-1.f, -1.f, +1.f, +1.f};
static const float  hip_side_sign[4] = {-1.f, +1.f, -1.f, +1.f};
static const float knee_side_sign[4] = {-.6333f, .6333f, -.7333f, .6333f};


static const float abad_offset[4] = {0.439f, 0.035f, -0.f, -0.038f};
static const float hip_offset[4]  = {1.531f, -1.494f, -1.440f, 1.524f};
static const float knee_offset[4] = {4.183f, -4.626f, -3.500f, 3.769f};


static inline uint16_t bswap16(uint16_t v){ return (uint16_t)((v>>8) | (v<<8)); }
static inline uint32_t xor_checksum_u32(const uint32_t* p, size_t n){ uint32_t s=0; for(size_t i=0;i<n;i++) s^=p[i]; return s; }
static inline float select_joint_value(const float abad[4], const float hip[4],
                                       const float knee[4], int joint, int leg){
  switch(joint){
    case 0: return abad[leg];
    case 1: return hip[leg];
    case 2: return knee[leg];
    default: return 0.f;
  }
}

static bool command_array_is_sane(const float* arr, size_t n){
  constexpr float kMaxAbsValue = 1e3f;
  for(size_t i=0;i<n;i++){
    const float v = arr[i];
    if (!std::isfinite(v) || std::fabs(v) > kMaxAbsValue){
      return false;
    }
  }
  return true;
}

static bool sanitize_spi_command(spi_command_t* cmd){
  const bool sane =
      command_array_is_sane(cmd->q_des_abad, 4) &&
      command_array_is_sane(cmd->q_des_hip, 4) &&
      command_array_is_sane(cmd->q_des_knee, 4) &&
      command_array_is_sane(cmd->qd_des_abad, 4) &&
      command_array_is_sane(cmd->qd_des_hip, 4) &&
      command_array_is_sane(cmd->qd_des_knee, 4) &&
      command_array_is_sane(cmd->kp_abad, 4) &&
      command_array_is_sane(cmd->kp_hip, 4) &&
      command_array_is_sane(cmd->kp_knee, 4) &&
      command_array_is_sane(cmd->kd_abad, 4) &&
      command_array_is_sane(cmd->kd_hip, 4) &&
      command_array_is_sane(cmd->kd_knee, 4) &&
      command_array_is_sane(cmd->tau_abad_ff, 4) &&
      command_array_is_sane(cmd->tau_hip_ff, 4) &&
      command_array_is_sane(cmd->tau_knee_ff, 4);

  if (!sane){
    std::memset(cmd, 0, sizeof(*cmd));
  }
  return sane;
}

static void fake_spine_control(spi_command_t* cmd, spi_data_t* data, spi_torque_t* tq, int b){
  tq->tau_abad[b] = cmd->kp_abad[b]*(cmd->q_des_abad[b]-data->q_abad[b])
                  + cmd->kd_abad[b]*(cmd->qd_des_abad[b]-data->qd_abad[b])
                  + cmd->tau_abad_ff[b];
  tq->tau_hip [b] = cmd->kp_hip [b]*(cmd->q_des_hip [b]-data->q_hip [b])
                  + cmd->kd_hip [b]*(cmd->qd_des_hip [b]-data->qd_hip [b])
                  + cmd->tau_hip_ff [b];
  tq->tau_knee[b] = cmd->kp_knee[b]*(cmd->q_des_knee[b]-data->q_knee[b])
                  + cmd->kd_knee[b]*(cmd->qd_des_knee[b]-data->qd_knee[b])
                  + cmd->tau_knee_ff[b];

  const float* lim = disabled_torque;
  if (cmd->flags[b] & 0x1) lim = (cmd->flags[b] & 0x2) ? wimp_torque : max_torque;
  tq->tau_abad[b] = std::max(-lim[0], std::min(lim[0], tq->tau_abad[b]));
  tq->tau_hip [b] = std::max(-lim[1], std::min(lim[1], tq->tau_hip [b]));
  tq->tau_knee[b] = std::max(-lim[2], std::min(lim[2], tq->tau_knee[b]));
}

static void spi_to_spine(spi_command_t* cmd, spine_cmd_t* scmd, int leg0){
  for (int i=0;i<2;i++){
    scmd->q_des_abad[i] = cmd->q_des_abad[i+leg0]*abad_side_sign[i+leg0] + abad_offset[i+leg0];
    scmd->q_des_hip [i] = cmd->q_des_hip [i+leg0]* hip_side_sign[i+leg0] +  hip_offset[i+leg0];
    scmd->q_des_knee[i] = cmd->q_des_knee[i+leg0]/knee_side_sign[i+leg0] + knee_offset[i+leg0];

    scmd->qd_des_abad[i]= cmd->qd_des_abad[i+leg0]*abad_side_sign[i+leg0];
    scmd->qd_des_hip [i]= cmd->qd_des_hip [i+leg0]* hip_side_sign[i+leg0];
    scmd->qd_des_knee[i]= cmd->qd_des_knee[i+leg0]/knee_side_sign[i+leg0];

    scmd->kp_abad[i] = cmd->kp_abad[i+leg0];
    scmd->kp_hip [i] = cmd->kp_hip [i+leg0];
    scmd->kp_knee[i] = cmd->kp_knee[i+leg0];
    scmd->kd_abad[i] = cmd->kd_abad[i+leg0];
    scmd->kd_hip [i] = cmd->kd_hip [i+leg0];
    scmd->kd_knee[i] = cmd->kd_knee[i+leg0];

    scmd->tau_abad_ff[i] = cmd->tau_abad_ff[i+leg0]*abad_side_sign[i+leg0];
    scmd->tau_hip_ff [i] = cmd->tau_hip_ff [i+leg0]* hip_side_sign[i+leg0];
    scmd->tau_knee_ff[i] = cmd->tau_knee_ff[i+leg0]*knee_side_sign[i+leg0];

    scmd->flags[i] = cmd->flags[i+leg0];
  }
  scmd->checksum = xor_checksum_u32(reinterpret_cast<uint32_t*>(scmd), 32);
}

static void apply_data_to_spi(spi_data_t* data,
                              const float* q_abad, const float* q_hip, const float* q_knee,
                              const float* qd_abad, const float* qd_hip, const float* qd_knee,
                              const uint16_t* flg, int leg0)
{
  for (int i=0;i<2;i++){
    const int leg = leg0 + i;
    data->q_abad[leg]  = (q_abad[i] - abad_offset[leg]) * abad_side_sign[leg];
    data->q_hip [leg]  = (q_hip [i] -  hip_offset[leg]) *  hip_side_sign[leg];
    data->q_knee[leg]  = (q_knee[i] - knee_offset[leg]) * knee_side_sign[leg];
    data->qd_abad[leg] = qd_abad[i] * abad_side_sign[leg];
    data->qd_hip [leg] = qd_hip [i] *  hip_side_sign[leg];
    data->qd_knee[leg] = qd_knee[i] * knee_side_sign[leg];
    data->flags[leg]   = flg[i];
  }
}

static int open_and_configure_spi(const char* path, int idx){
  int fd = open(path, O_RDWR | O_CLOEXEC);
  if (fd < 0){ LOG_ERROR("[ERROR] open {}: {}", path, std::strerror(errno)); return -1; }

  if (ioctl(fd, SPI_IOC_WR_LSB_FIRST, &g_lsb_first) < 0) LOG_WARN("[WARN ] WR_LSB_FIRST: {}", std::strerror(errno));
  if (ioctl(fd, SPI_IOC_RD_LSB_FIRST, &g_lsb_first) < 0) LOG_WARN("[WARN ] RD_LSB_FIRST: {}", std::strerror(errno));

  if (ioctl(fd, SPI_IOC_WR_MODE, &g_mode) < 0) LOG_ERROR("[ERROR] WR_MODE: {}", std::strerror(errno));
  if (ioctl(fd, SPI_IOC_RD_MODE, &g_mode) < 0) LOG_ERROR("[ERROR] RD_MODE: {}", std::strerror(errno));

  unsigned int hz = MINC_SPI_HZ;
  if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz) < 0) LOG_ERROR("[ERROR] WR_MAX_SPEED_HZ: {}", std::strerror(errno));
  if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &hz) < 0) LOG_ERROR("[ERROR] RD_MAX_SPEED_HZ: {}", std::strerror(errno));

  uint8_t bpw = (uint8_t)WANT_BPW;
  if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bpw) < 0 || ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bpw) < 0){
    bpw = 8;
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bpw) < 0 || ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bpw) < 0){
      LOG_ERROR("[ERROR] cannot set bpw to {} or 8 on {}", (unsigned)WANT_BPW, path);
      close(fd); return -1;
    }
  }
  g_real_bpw[idx] = bpw;
  return fd;
}

// ===== 对外接口 =====
int spi_open(){
  LOG_INFO("[RT SPI] Board 0 using {}", SPI_DEV0_PATH);
  LOG_INFO("[RT SPI] Board 1 using {}", SPI_DEV1_PATH);
  g_fd[0] = open_and_configure_spi(SPI_DEV0_PATH, 0);
  g_fd[1] = open_and_configure_spi(SPI_DEV1_PATH, 1);
  if (g_fd[0] < 0 || g_fd[1] < 0) return -1;
  return 0;
}

void init_spi(){
  std::memset(&spi_command_drv, 0, sizeof(spi_command_drv));
  std::memset(&spi_data_drv,    0, sizeof(spi_data_drv));
  if (pthread_mutex_init(&g_spi_mutex, nullptr) != 0)
    LOG_ERROR("[RT SPI] Failed to create spi data mutex");

#ifdef K_EXPECTED_COMMAND_SIZE
  { size_t s = sizeof(spi_command_t);
    if (s != K_EXPECTED_COMMAND_SIZE) LOG_ERROR("[RT SPI] Error command size {} != {}", s, K_EXPECTED_COMMAND_SIZE);
    else                              LOG_INFO("[RT SPI] command size good");
  }
#endif
#ifdef K_EXPECTED_DATA_SIZE
  { size_t s = sizeof(spi_data_t);
    if (s != K_EXPECTED_DATA_SIZE) LOG_ERROR("[RT SPI] Error data size {} != {}", s, K_EXPECTED_DATA_SIZE);
    else                            LOG_INFO("[RT SPI] data size good");
  }
#endif

  (void)spi_open();
}

void spi_driver_run(){
  bool cmd_ok = sanitize_spi_command(&spi_command_drv);
  if (!cmd_ok && !g_warned_bad_spi_cmd){
    LOG_WARN("[SPI-CMD] detected invalid/uninitialized payload, zeroed until controller updates");
    g_warned_bad_spi_cmd = true;
  } else if (cmd_ok && g_warned_bad_spi_cmd){
    LOG_INFO("[SPI-CMD] command payload is now valid");
    g_warned_bad_spi_cmd = false;
  }

  static int s_spi_cmd_log_decimator = 0;
  if ((s_spi_cmd_log_decimator++ % 500) == 0){
    const char* joint_name[3] = {"abad", "hip", "knee"};
    for (int leg = 0; leg < 4; ++leg){
      LOG_INFO("[SPI-CMD] leg{} flags={:#010x}", leg, spi_command_drv.flags[leg]);
      for (int joint = 0; joint < 3; ++joint){
        float qdes = select_joint_value(spi_command_drv.q_des_abad,
                                        spi_command_drv.q_des_hip,
                                        spi_command_drv.q_des_knee,
                                        joint, leg);
        float qddes = select_joint_value(spi_command_drv.qd_des_abad,
                                         spi_command_drv.qd_des_hip,
                                         spi_command_drv.qd_des_knee,
                                         joint, leg);
        float kp = select_joint_value(spi_command_drv.kp_abad,
                                      spi_command_drv.kp_hip,
                                      spi_command_drv.kp_knee,
                                      joint, leg);
        float kd = select_joint_value(spi_command_drv.kd_abad,
                                      spi_command_drv.kd_hip,
                                      spi_command_drv.kd_knee,
                                      joint, leg);
        float tau = select_joint_value(spi_command_drv.tau_abad_ff,
                                       spi_command_drv.tau_hip_ff,
                                       spi_command_drv.tau_knee_ff,
                                       joint, leg);
        LOG_INFO("  [joint:{}] qDes={:.3f} qdDes={:.3f} Kp={:.2f} Kd={:.2f} tau={:.2f}",
                    joint_name[joint], qdes, qddes, kp, kd, tau);
      }
    }
  }

  for (int i=0;i<4;i++) fake_spine_control(&spi_command_drv, &spi_data_drv, &spi_torque, i);
  pthread_mutex_lock(&g_spi_mutex);
  spi_send_receive(&spi_command_drv, &spi_data_drv);
  pthread_mutex_unlock(&g_spi_mutex);
}

spi_command_t* get_spi_command(){ return &spi_command_drv; }
spi_data_t*    get_spi_data()   { return &spi_data_drv;  }

// ===== 低层 xfer =====
static int spi_xfer_frame(int fd, int idx, uint16_t* tx16, uint16_t* rx16, size_t words){
  struct spi_ioc_transfer tr{};
  tr.speed_hz      = MINC_SPI_HZ;
  tr.bits_per_word = g_real_bpw[idx];
  tr.cs_change     = 0;
  tr.delay_usecs   = SPI_FRAME_DELAY_US;
  tr.len           = (uint32_t)(2 * words);

  if (g_real_bpw[idx] == 16){
    tr.tx_buf = (uint64_t)tx16;
    tr.rx_buf = (uint64_t)rx16;
    return ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
  } else {
    static uint16_t tx_swap[K_WORDS_PER_MESSAGE];
    for (size_t i=0;i<words;i++) tx_swap[i] = bswap16(tx16[i]); // 8-bit 路径：TX 先交换
    tr.tx_buf = (uint64_t)tx_swap;
    tr.rx_buf = (uint64_t)rx16;
    int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret >= 0){
      for (size_t i=0;i<words; i++) rx16[i] = bswap16(rx16[i]); // RX 也需要整帧换序
    }
    return ret;
  }
}

// ===== 关键：发送/接收 + 校验/解包 =====
void spi_send_receive(spi_command_t* command, spi_data_t* data){
  static int spi_driver_iterations = 0;
  spi_driver_iterations++;
  data->spi_driver_status = spi_driver_iterations << 16;

  uint16_t tx_buf[K_WORDS_PER_MESSAGE]{}; // 66
  uint16_t rx_buf[K_WORDS_PER_MESSAGE]{}; // 66

  for (int b=0; b<2; b++){
    std::memset(rx_buf, 0, sizeof(rx_buf));
    // 打包两条腿命令
    spi_to_spine(command, &g_spine_cmd, b*2);
    std::memcpy(tx_buf, reinterpret_cast<uint16_t*>(&g_spine_cmd), sizeof(g_spine_cmd));

    const int fd = g_fd[b];
    if (fd < 0) continue;

    int ret = spi_xfer_frame(fd, b, tx_buf, rx_buf, K_WORDS_PER_MESSAGE);
    if (ret < 0){
      LOG_ERROR("[ERROR] SPI_IOC_MESSAGE: {}", std::strerror(errno));
      continue;
    }
    if (ret < (int)(2 * K_WORDS_PER_MESSAGE)){
      LOG_ERROR("[ERROR] SPI_IOC_MESSAGE short transfer: {}/ {} bytes",
                   ret, sizeof(uint16_t) * (size_t)K_WORDS_PER_MESSAGE);
      continue;
    }

    // 仅以前 30×16bit 做校验（rx_buf[0..29] 已在 8-bit 路径下交换到小端半字视图）
    uint32_t calc_lohi = 0, calc_hilo = 0;
    for (int i=0;i<14;i++){
      uint16_t lo = rx_buf[2*i+0];
      uint16_t hi = rx_buf[2*i+1];
      calc_lohi ^= ((uint32_t)hi << 16) | lo;  // lo 在前
      calc_hilo ^= ((uint32_t)lo << 16) | hi;  // 反半字序
    }
    const uint32_t wire_lohi = ((uint32_t)rx_buf[29] << 16) | rx_buf[28];
    const uint32_t wire_hilo = ((uint32_t)rx_buf[28] << 16) | rx_buf[29];

    // 首帧探测半字序
    if (g_half_order[b] < 0){
      if      (calc_lohi == wire_lohi) g_half_order[b] = 0;
      else if (calc_hilo == wire_hilo) g_half_order[b] = 1;
    }

#if SPI_DEBUG
    g_frame_cnt[b]++;
    if (SPI_DUMP_EVERY > 0 && (g_frame_cnt[b] % SPI_DUMP_EVERY) == 0){
      DBG("[SPI][board %d][bpw=%u][Hz=%u][delay=%u us] frame=%" PRIu64 "\n",
          b, (unsigned)g_real_bpw[b], (unsigned)MINC_SPI_HZ, (unsigned)SPI_FRAME_DELAY_US, g_frame_cnt[b]);
      DBG("  rx[0..7]= ");
      for (int i=0;i<8;i++) DBG("%04x ", rx_buf[i]);
      DBG("\n  rx[26..29]= %04x %04x %04x %04x\n", rx_buf[26], rx_buf[27], rx_buf[28], rx_buf[29]);
      DBG("  calc_lohi=0x%08x wire_lohi=0x%08x\n", calc_lohi, wire_lohi);
      DBG("  calc_hilo=0x%08x wire_hilo=0x%08x\n", calc_hilo, wire_hilo);
      DBG("  half_order=%d (0=hi<<16|lo, 1=lo<<16|hi)\n", g_half_order[b]);
    }
#endif

    bool ok = false;
    if      (g_half_order[b]==0 && calc_lohi==wire_lohi) ok = true;
    else if (g_half_order[b]==1 && calc_hilo==wire_hilo) ok = true;
    else if (g_half_order[b]< 0 && (calc_lohi==wire_lohi || calc_hilo==wire_hilo)) ok = true;

    if (!ok){
      g_checksum_failures[b]++;
      if (g_checksum_failures[b] == 1 || (g_checksum_failures[b] % 50) == 0){
        LOG_ERROR("[SPI][board {}] checksum failure #{} (iter={} last_good_iter={})",
                    b,
                    g_checksum_failures[b],
                    spi_driver_iterations,
                    g_last_good_iter[b]);
      }
      LOG_ERROR("SPI ERROR BAD CHECKSUM (board {}) lohi:calc={:#010x} wire={:#010x} | hilo:calc={:#010x} wire={:#010x}",
                  b, calc_lohi, wire_lohi, calc_hilo, wire_hilo);
      continue; // 丢帧保护：本帧不更新 data
    }
#if SPI_DEBUG
    DBG("  [SPI][board %d] checksum ok (order=%d, frame=%" PRIu64 ")\n",
        b, g_half_order[b], g_frame_cnt[b]);
#endif

    // 校验通过：按半字序解包 12 个 float + flags
    if (g_checksum_failures[b] > 0){
      LOG_WARN("[SPI][board {}] checksum recovered after {} failed frame(s) at iter={}",
                  b,
                  g_checksum_failures[b],
                  spi_driver_iterations);
      g_checksum_failures[b] = 0;
    }
    g_last_good_iter[b] = spi_driver_iterations;
    auto u32_to_f = [](uint32_t u){ float f; std::memcpy(&f,&u,sizeof(float)); return f; };
    auto get_u32  = [&](int i)->uint32_t{
      uint16_t lo = rx_buf[2*i+0], hi = rx_buf[2*i+1];
      if (g_half_order[b] == 1) std::swap(lo, hi);
      return ((uint32_t)hi << 16) | lo;
    };

    float    s_q_abad[2] = { u32_to_f(get_u32(0)),  u32_to_f(get_u32(1))  };
    float    s_q_hip [2] = { u32_to_f(get_u32(2)),  u32_to_f(get_u32(3))  };
    float    s_q_knee[2] = { u32_to_f(get_u32(4)),  u32_to_f(get_u32(5))  };
    float   s_qd_abad[2] = { u32_to_f(get_u32(6)),  u32_to_f(get_u32(7))  };
    float   s_qd_hip [2] = { u32_to_f(get_u32(8)),  u32_to_f(get_u32(9))  };
    float   s_qd_knee[2] = { u32_to_f(get_u32(10)), u32_to_f(get_u32(11)) };
    uint32_t w12         = get_u32(12);
    uint16_t s_flags[2]  = { (uint16_t)(w12 & 0xFFFF), (uint16_t)((w12 >> 16) & 0xFFFF) };

    apply_data_to_spi(&spi_data_drv, s_q_abad, s_q_hip, s_q_knee, s_qd_abad, s_qd_hip, s_qd_knee, s_flags, b*2);
  }

  static int s_spi_rx_log_decimator = 0;
  if ((s_spi_rx_log_decimator++ % 500) == 0){
    for (int leg = 0; leg < 4; ++leg){
      float qA = data->q_abad[leg];
      float qH = data->q_hip[leg];
      float qK = data->q_knee[leg];
      float qdA = data->qd_abad[leg];
      float qdH = data->qd_hip[leg];
      float qdK = data->qd_knee[leg];
      LOG_INFO("[SPI-RX ] leg{} q=[{:.3f} {:.3f} {:.3f}] qd=[{:.3f} {:.3f} {:.3f}] flags={:#010x}",
                  leg, qA, qH, qK, qdA, qdH, qdK, data->flags[leg]);
    }
  }
}

#endif // __linux__
