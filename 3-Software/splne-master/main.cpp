// main.cpp — Spine（SPI 从机 + CAN 桥）完整修复版
#include "mbed.h"
#include "math_ops.h"
#include <cstring>
#include "leg_message.h"

// 文件概览：腿控板作为 SPI 从机接收主机命令并通过两路 CAN 驱动电机，同时返回状态。
// Google 风格：简要说明文件职责，便于快速理解模块边界。

// -------------------- 配置 --------------------
#define RX_LEN   66    // SPI 收包 16bit 字数
#define TX_LEN   66    // SPI 发包 16bit 字数

#define DATA_LEN 30    // 30 x 16bit = 60B（12 个 float + flags + 32-bit 校验）
#define CMD_LEN  66    // 66 x 16bit（32 个 u32 + 32-bit 校验）

#define CAN_ID   0x0

/// Value Limits（物理约束） ///
#define P_MIN -12.5f
#define P_MAX  12.5f
#define V_MIN -65.0f
#define V_MAX  65.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -23.5f
#define T_MAX  23.5f

/// Joint Soft Stops（软限位） ///
#define A_LIM_P 1.5f
#define A_LIM_N -1.5f
#define H_LIM_P 5.0f
#define H_LIM_N -5.0f
#define K_LIM_P 0.2f
#define K_LIM_N 7.7f
#define KP_SOFTSTOP 100.0f
#define KD_SOFTSTOP 0.4f;

#define STRICT_DROP_ON_CRC_FAIL 1   // CRC 失败是否直接丢弃命令
#define CUT_MOTOR_ON_BAD_SPI    1   // 连续 CRC 失败是否触发断电
#define SPI_BAD_STREAK_MAX      5   // 触发断电的连续失败阈值

// -------------------- 全局对象 --------------------
spi_data_t    spi_data;         // spine -> host：回传的两腿状态与校验
spi_command_t spi_command;      // host  -> spine：经校验后的最新指令
static spi_command_t spi_rx_shadow;    // SPI ISR 暂存的命令，待主循环搬运

static uint16_t rx_buff[RX_LEN];       // SPI 半字接收缓存
static uint16_t tx_buff[TX_LEN];       // SPI 半字发送缓存

DigitalOut led(PC_5);                   // 急停指示灯（亮=急停触发）
Serial     pc(PA_2, PA_3);              // 上位机调试串口

// 与硬件一致的 CAN 引脚
CAN can1(PB_12, PB_13, 1000000);        // CAN1：第一条腿
CAN can2(PB_8,  PB_9,  1000000);        // CAN2：第二条腿

CANMessage rxMsg1, rxMsg2;              // CAN 反馈暂存
CANMessage a1_can, a2_can, h1_can, h2_can, k1_can, k2_can; // 六个关节的命令帧

InterruptIn cs(PA_4);                   // SPI 片选，下降沿触发 ISR
DigitalIn   estop(PB_15);               // 硬件急停输入（上拉，低有效）

leg_state   l1_state, l2_state;         // 两条腿当前状态
leg_control l1_control, l2_control;     // 两条腿待发送的控制量

volatile int enabled = 0;              // 当前是否已进入力矩模式
volatile uint32_t g_spi_frames = 0;    // SPI 已处理的帧数
volatile uint32_t g_cmd_bad    = 0;    // CRC 失败的帧数
volatile int      g_last_len   = 0;    // 上一帧实际接收的半字数
static volatile int spi_ready  = 0;    // SPI 是否完成初始化
static volatile int g_cmd_pending = 0; // 是否有待处理的 SPI 命令帧
static volatile int g_need_cut_motors = 0; // 是否需要立即退出力矩模式
static volatile int g_crc_bad_streak = 0;  // 连续 CRC 失败次数
static volatile int counter2 = 0;      // 站立模式计数器（预留）
static volatile int is_standing = 0;   // 站立模式标志位

// 将 TX 缓冲区的第一个 16bit 预写入 SPI DR，确保片选拉低时立即输出。
static inline void spi_prime_first_word(void){
  if(!spi_ready) return;
  SPI1->DR = tx_buff[0];
}

// 计算 n 个 32-bit 字的逐字 XOR，用于 SPI 命令校验。
static inline uint32_t xor_checksum_u32(const uint32_t* p, size_t n){
  uint32_t s = 0;
  for(size_t i = 0; i < n; i++) {
    s ^= p[i];
  }
  return s;
}

// 根据 TX buffer 前 30 个 16-bit 半字生成 32-bit 序列（低 16bit 在前）并计算 XOR 校验。
static inline uint32_t checksum_from_tx_words30(const uint16_t* w16){
  uint32_t s = 0;
  for (int i = 0; i < 14; i++){
    uint32_t lo = w16[2*i + 0];
    uint32_t hi = w16[2*i + 1];
    s ^= (hi << 16) | lo;  // 与 Jetson 端 calc_lohi 一致
  }
  return s;
}

// ===== CAN 打包/解包 =====
// 将单关节控制量压缩为 8 字节 CAN 帧，遵循电机驱动的打包协议。
void pack_cmd(CANMessage * msg, joint_control joint){
  // 期望值按物理上限限幅，防止驱动接收异常值。
  float p_des = fminf(fmaxf(P_MIN, joint.p_des), P_MAX);
  float v_des = fminf(fmaxf(V_MIN, joint.v_des), V_MAX);
  float kp    = fminf(fmaxf(KP_MIN, joint.kp),  KP_MAX);
  float kd    = fminf(fmaxf(KD_MIN, joint.kd),  KD_MAX);
  float t_ff  = fminf(fmaxf(T_MIN, joint.t_ff), T_MAX);

  uint16_t p_int  = float_to_uint(p_des, P_MIN, P_MAX, 16);
  uint16_t v_int  = float_to_uint(v_des, V_MIN, V_MAX, 12);
  uint16_t kp_int = float_to_uint(kp,    KP_MIN, KP_MAX, 12);
  uint16_t kd_int = float_to_uint(kd,    KD_MIN, KD_MAX, 12);
  uint16_t t_int  = float_to_uint(t_ff,  T_MIN,  T_MAX,  12);

  msg->data[0] = p_int>>8;
  msg->data[1] = p_int&0xFF;
  msg->data[2] = v_int>>4;
  msg->data[3] = ((v_int&0xF)<<4)|(kp_int>>8);
  msg->data[4] = kp_int&0xFF;
  msg->data[5] = kd_int>>4;
  msg->data[6] = ((kd_int&0xF)<<4)|(t_int>>8);
  msg->data[7] = t_int&0xff;
  msg->len     = 8;
}
// 将电机驱动返回的 8 字节状态帧解包到指定腿的状态结构体。
void unpack_reply(CANMessage msg, leg_state * leg){
  uint16_t id    = msg.data[0];
  uint16_t p_int = (msg.data[1]<<8)|msg.data[2];
  uint16_t v_int = (msg.data[3]<<4)|(msg.data[4]>>4);
  uint16_t i_int = ((msg.data[4]&0xF)<<8)|msg.data[5];

  float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
  float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
  float t = uint_to_float(i_int, -T_MAX, T_MAX, 12);

  // 1:髋外展，2:髋，3:膝；写入对应腿状态。
  if(id==1){      leg->a.p = p; leg->a.v = v; leg->a.t = t; }
  else if(id==2){ leg->h.p = p; leg->h.v = v; leg->h.t = t; }
  else if(id==3){ leg->k.p = p; leg->k.v = v; leg->k.t = t; }
}

// 发一轮 6 条 CAN（两腿）
// 将每个关节的期望控制量打包，并设置对应的报文 ID。
void PackAll(){
  pack_cmd(&a1_can, l1_control.a);
  pack_cmd(&a2_can, l2_control.a);
  pack_cmd(&h1_can, l1_control.h);
  pack_cmd(&h2_can, l2_control.h);
  pack_cmd(&k1_can, l1_control.k);
  pack_cmd(&k2_can, l2_control.k);

  a1_can.id = a2_can.id = 0x1;
  h1_can.id = h2_can.id = 0x2;
  k1_can.id = k2_can.id = 0x3;
}
// 依次向两条 CAN 总线写入关节命令，150us 间隔避免突发阻塞。
void WriteAll(){
  can1.write(a1_can); wait_us(150);
  can2.write(a2_can); wait_us(150);
  can1.write(h1_can); wait_us(150);
  can2.write(h2_can); wait_us(150);
  can1.write(k1_can); wait_us(150);
  can2.write(k2_can); wait_us(150);
}

// 电机模式控制
// Zero: 发送 0xFF...0xFE，让驱动清零偏置（立即广播一次）。
void Zero(CANMessage * msg){ for (int i=0;i<7;i++) msg->data[i]=0xFF; msg->data[7]=0xFE; msg->len=8; WriteAll(); }
// EnterMotorMode: 0xFF...0xFC，进入力矩/闭环模式。
void EnterMotorMode(CANMessage * msg){ for (int i=0;i<7;i++) msg->data[i]=0xFF; msg->data[7]=0xFC; msg->len=8; }
// ExitMotorMode: 0xFF...0xFD，退出力矩模式并松刹车。
void ExitMotorMode (CANMessage * msg){ for (int i=0;i<7;i++) msg->data[i]=0xFF; msg->data[7]=0xFD; msg->len=8; }


// 串口中断：处理调试键盘指令，快速切换电机模式或站立状态。
void serial_isr(){
     /// 串口调试指令处理 ///
     while(pc.readable()){
        char c = pc.getc();
        //led = !led;
        // 调试快捷键：Esc 退出力矩，m 进入力矩，s 站立模式，z 零点校准。
        switch(c){
            case(27):
                //loop.detach();
                pc.printf("\n\r exiting motor mode \n\r");
                ExitMotorMode(&a1_can);
                ExitMotorMode(&a2_can);
                ExitMotorMode(&h1_can);
                ExitMotorMode(&h2_can);
                ExitMotorMode(&k1_can);
                ExitMotorMode(&k2_can);
                enabled = 0;
                break;
            case('m'):
                pc.printf("\n\r entering motor mode \n\r");
                EnterMotorMode(&a1_can);
                EnterMotorMode(&a2_can);
                EnterMotorMode(&h1_can);
                EnterMotorMode(&h2_can);
                EnterMotorMode(&k1_can);
                EnterMotorMode(&k2_can);
                wait(.5);
                enabled = 1;
                //loop.attach(&sendCMD, .001);
                break;
            case('s'):
                pc.printf("\n\r standing \n\r");
                counter2 = 0;
                is_standing = 1;
                //stand();
                break;
            case('z'):
                pc.printf("\n\r zeroing \n\r");
                Zero(&a1_can);
                Zero(&a2_can);
                Zero(&h1_can);
                Zero(&h2_can);
                Zero(&k1_can);
                Zero(&k2_can);
                break;
            }
        }
        WriteAll();
        
    }

// 软限位：超出阈值时清零速度与比例增益，并用固定 KD+力矩将关节推回安全区。返回 1 表示已触发。
int softstop_joint(joint_state s, joint_control * c, float limit_p, float limit_n){
  if(s.p >= limit_p){
    c->v_des = 0; c->kp = 0; c->kd = KD_SOFTSTOP; c->t_ff += KP_SOFTSTOP*(limit_p - s.p); return 1;
  } else if(s.p <= limit_n){
    c->v_des = 0; c->kp = 0; c->kd = KD_SOFTSTOP; c->t_ff += KP_SOFTSTOP*(limit_n - s.p); return 1;
  }
  return 0;
}

// 控制主逻辑：根据最新的 spi_command 生成 l1/l2 控制，把反馈写入 spi_data，并构造 tx_buff。
// 步骤：1) 根据 flags 切换力矩模式；2) 将 CAN 反馈写入 spi_data；
//      3) 急停时清零并打出标志；4) 正常时填充主机期望并做软限位；5) 打包 + 校验到 tx_buff。
void control_and_build_tx(){
  // 入/退力矩模式
  const uint32_t torque_en = (spi_command.flags[0] | spi_command.flags[1]) & 0x1;
  if((torque_en == 1) && (enabled==0)){
    enabled = 1;
    EnterMotorMode(&a1_can); can1.write(a1_can);
    EnterMotorMode(&a2_can); can2.write(a2_can);
    EnterMotorMode(&k1_can); can1.write(k1_can);
    EnterMotorMode(&k2_can); can2.write(k2_can);
    EnterMotorMode(&h1_can); can1.write(h1_can);
    EnterMotorMode(&h2_can); can2.write(h2_can);
    pc.printf("[MOTOR] enter torque mode (estop=%d)\n\r", estop.read());
  } else if((torque_en == 0) && (enabled==1)){
    enabled = 0;
    ExitMotorMode(&a1_can); can1.write(a1_can);
    ExitMotorMode(&a2_can); can2.write(a2_can);
    ExitMotorMode(&h1_can); can1.write(h1_can);
    ExitMotorMode(&h2_can); can2.write(h2_can);
    ExitMotorMode(&k1_can); can1.write(k1_can);
    ExitMotorMode(&k2_can); can2.write(k2_can);
    pc.printf("[MOTOR] exit torque mode\n\r");
  }

  // CAN -> spi_data（host 会做坐标还原）
  spi_data.q_abad[0] = l1_state.a.p;  spi_data.q_abad[1] = l2_state.a.p;
  spi_data.q_hip [0] = l1_state.h.p;  spi_data.q_hip [1] = l2_state.h.p;
  spi_data.q_knee[0] = l1_state.k.p;  spi_data.q_knee[1] = l2_state.k.p;
  spi_data.qd_abad[0]= l1_state.a.v;  spi_data.qd_abad[1]= l2_state.a.v;
  spi_data.qd_hip [0]= l1_state.h.v;  spi_data.qd_hip [1]= l2_state.h.v;
  spi_data.qd_knee[0]= l1_state.k.v;  spi_data.qd_knee[1]= l2_state.k.v;

  // 急停按下：清零控制输出，写入特殊标志并点亮 LED。
  if(estop==0){
    memset(&l1_control, 0, sizeof(l1_control));
    memset(&l2_control, 0, sizeof(l2_control));
    spi_data.flags[0] = 0xDEAD;  // 用特殊码提醒上位机：急停触发
    spi_data.flags[1] = 0xDEAD;
    led = 1;
  } else {
    // 正常工作：刷新期望、应用软限位，关闭 LED。
    led = 0;
    memset(&l1_control, 0, sizeof(l1_control));
    memset(&l2_control, 0, sizeof(l2_control));

    // 主机发送的期望值（按既定 SPI 协议顺序）。
    l1_control.a.p_des = spi_command.q_des_abad[0];
    l1_control.a.v_des = spi_command.qd_des_abad[0];
    l1_control.a.kp    = spi_command.kp_abad[0];
    l1_control.a.kd    = spi_command.kd_abad[0];
    l1_control.a.t_ff  = spi_command.tau_abad_ff[0];

    l1_control.h.p_des = spi_command.q_des_hip[0];
    l1_control.h.v_des = spi_command.qd_des_hip[0];
    l1_control.h.kp    = spi_command.kp_hip[0];
    l1_control.h.kd    = spi_command.kd_hip[0];
    l1_control.h.t_ff  = spi_command.tau_hip_ff[0];

    l1_control.k.p_des = spi_command.q_des_knee[0];
    l1_control.k.v_des = spi_command.qd_des_knee[0];
    l1_control.k.kp    = spi_command.kp_knee[0];
    l1_control.k.kd    = spi_command.kd_knee[0];
    l1_control.k.t_ff  = spi_command.tau_knee_ff[0];

    l2_control.a.p_des = spi_command.q_des_abad[1];
    l2_control.a.v_des = spi_command.qd_des_abad[1];
    l2_control.a.kp    = spi_command.kp_abad[1];
    l2_control.a.kd    = spi_command.kd_abad[1];
    l2_control.a.t_ff  = spi_command.tau_abad_ff[1];

    l2_control.h.p_des = spi_command.q_des_hip[1];
    l2_control.h.v_des = spi_command.qd_des_hip[1];
    l2_control.h.kp    = spi_command.kp_hip[1];
    l2_control.h.kd    = spi_command.kd_hip[1];
    l2_control.h.t_ff  = spi_command.tau_hip_ff[1];

    l2_control.k.p_des = spi_command.q_des_knee[1];
    l2_control.k.v_des = spi_command.qd_des_knee[1];
    l2_control.k.kp    = spi_command.kp_knee[1];
    l2_control.k.kd    = spi_command.kd_knee[1];
    l2_control.k.t_ff  = spi_command.tau_knee_ff[1];

    // 软限位 -> flags
    spi_data.flags[0]  = 0;
    spi_data.flags[0] |= softstop_joint(l1_state.a, &l1_control.a, A_LIM_P, A_LIM_N);
    spi_data.flags[0] |= (softstop_joint(l1_state.h, &l1_control.h, H_LIM_P, H_LIM_N))<<1;
    //spi_data.flags[0] |= (softstop_joint(l1_state.k, &l1_control.k, K_LIM_P, K_LIM_N))<<2;

    spi_data.flags[1]  = 0;
    spi_data.flags[1] |= softstop_joint(l2_state.a, &l2_control.a, A_LIM_P, A_LIM_N);
    spi_data.flags[1] |= (softstop_joint(l2_state.h, &l2_control.h, H_LIM_P, H_LIM_N))<<1;
    //spi_data.flags[1] |= (softstop_joint(l2_state.k, &l2_control.k, K_LIM_P, K_LIM_N))<<2;
  }

  // ===== 关键修复：按“线上半字流”计算校验，并回填到 tx_buff[28..29] =====
  // 先把有效负载(28 半字 = 14×u32)写入 tx_buff[0..27]
  for (int i = 0; i < 28; i++) {
    tx_buff[i] = ((uint16_t*)(&spi_data))[i];
  }
  // 基于“将要上线”的 28 半字计算 32-bit XOR
  uint32_t chk = checksum_from_tx_words30(tx_buff);
  tx_buff[28]  = (uint16_t)(chk & 0xFFFF);      // 低 16
  tx_buff[29]  = (uint16_t)(chk >> 16);         // 高 16
  spi_data.checksum = chk;                      // 仅作串口观测（线上以 tx_buff 为准）
  for (int i = 30; i < TX_LEN; i++) tx_buff[i] = 0;  // 清理尾部

  // 将首个 16bit 预写入 SPI1->DR，确保主机采到当前帧。
  spi_prime_first_word();

}

// ===== SPI 中断（全双工搬运）=====
// 片选为低期间：TXE 时塞下一个半字，RXNE 时收主机半字；结束后校验长度与 XOR。
// 校验通过则将 spi_rx_shadow 搬到命令缓冲并置 g_cmd_pending，交由主循环消费。
void spi_isr(void)
{
  int rx_i = 0;
  int tx_i = 1;  // tx_buff[0] 已由 spi_prime_first_word 预写入

  while (cs == 0) {
    uint32_t sr = SPI1->SR;

    if ((sr & SPI_SR_TXE) && (tx_i < TX_LEN)) {
      SPI1->DR = tx_buff[tx_i++];
    }
    if (sr & SPI_SR_RXNE) {
      if (rx_i < RX_LEN) {
        rx_buff[rx_i++] = SPI1->DR;
      } else {
        volatile uint16_t dump = SPI1->DR;
        (void)dump;
      }
    }
  }
  g_last_len = rx_i;

  // 处理溢出：读 DR + SR 清除 OVR 标志，避免下一帧异常。
  if (SPI1->SR & SPI_SR_OVR) {
    volatile uint16_t dump = SPI1->DR;
    (void)dump;
    dump = SPI1->SR;
    (void)dump;
  }
  // 校验 host 命令（按线上 32×u32 = 64 半字 + checksum 的前 32×u32 计算）
  const bool len_ok = (g_last_len == CMD_LEN);
  uint32_t calc_checksum = 0;
  if(len_ok){
    for(int i = 0; i < CMD_LEN; i++){
      ((uint16_t*)(&spi_rx_shadow))[i] = rx_buff[i];
    }
    calc_checksum = xor_checksum_u32((uint32_t*)rx_buff, 32);
  }

  // 严格 CRC：长度或校验失败则丢帧，必要时累计错误触发断电。
  if (!len_ok || (spi_rx_shadow.checksum != calc_checksum)) {
#if STRICT_DROP_ON_CRC_FAIL
    g_cmd_bad++;
#if CUT_MOTOR_ON_BAD_SPI
    if(++g_crc_bad_streak >= SPI_BAD_STREAK_MAX){
      g_need_cut_motors = 1;
    }
#endif
    // 不更新 spi_command/tx_buff，只返回，继续发送上一帧缓存。
    g_spi_frames++;
    spi_prime_first_word();
    return;
#else
    // 宽松模式：即便 CRC 失败也放行（仅调试用）。
#endif
  }

  g_crc_bad_streak = 0;

  // CRC pass: notify main loop to process command
  g_cmd_pending = 1;

  g_spi_frames++;
}

// ===== SPI 初始化 =====
// 配置 16bit Mode0 从机，绑定 CS 下降沿触发 ISR，并预写首个半字。
void init_spi(void){
  pc.printf("SPI Init ...\n\r");
  SPISlave *spi = new SPISlave(PA_7, PA_6, PA_5, PA_4); // MOSI, MISO, SCK, CS
  spi->format(16, 0);       // 16bit, Mode 0
  spi->frequency(5000000);  // 与 Jetson 保持 1MHz
  spi->reply(0xff);
  cs.fall(&spi_isr);
  spi_ready = 1;
  spi_prime_first_word();
  pc.printf("SPI Init done\n\r");
}

// ===== 主入口 =====
// 初始化串口/CAN/SPI，先构造一帧空数据，随后主循环依次：
// 1) 轮询 CAN 反馈；2) 处理 SPI 新命令（如有）；3) 定期输出统计。
int main() {
  wait(1);  // 上电等待，确保驱动和主机稳定
  pc.baud(921600);
  pc.attach(&serial_isr, Serial::RxIrq); // 绑定串口接收中断，处理键盘指令（含 z -> zeroing）
  estop.mode(PullUp);

  // 允许所有标准帧，便于驱动通信与调试。
  can1.filter(0, 0, CANStandard, 0);
  can2.filter(0, 0, CANStandard, 0);

  memset(&tx_buff,   0, sizeof(tx_buff));
  memset(&spi_data,  0, sizeof(spi_data));
  memset(&spi_command, 0, sizeof(spi_command));
  memset(&spi_rx_shadow,0, sizeof(spi_rx_shadow));

  a1_can.len = a2_can.len = h1_can.len = h2_can.len = k1_can.len = k2_can.len = 8; // 关节命令帧 8 字节
  rxMsg1.len = rxMsg2.len = 6;  // 驱动反馈帧 6 字节

  a1_can.id = a2_can.id = 0x1;
  h1_can.id = h2_can.id = 0x2;
  k1_can.id = k2_can.id = 0x3;

  // 先构建一个初始 tx 帧，避免首帧全 0
  control_and_build_tx();

  // SPI 初始化（等待 CS 释放避免低电平期间初始化）
  while(cs.read() == 0){ wait_us(10); }
  init_spi();

  // 500ms 周期打印轻量状态
  Timer t; t.start();
  uint32_t last_ms = 0, last_frames = 0, last_bad = 0;

  while(1) {
    // 1) 轮询 CAN 反馈。
    if (can2.read(rxMsg2)) unpack_reply(rxMsg2, &l2_state);
    if (can1.read(rxMsg1)) unpack_reply(rxMsg1, &l1_state);
    wait_us(50);

    // 2) CRC 连续异常时的保护：必要时立即退出力矩。
    if (g_need_cut_motors) {
      g_need_cut_motors = 0;
#if CUT_MOTOR_ON_BAD_SPI
      if (enabled) {
        ExitMotorMode(&a1_can); can1.write(a1_can);
        ExitMotorMode(&a2_can); can2.write(a2_can);
        ExitMotorMode(&h1_can); can1.write(h1_can);
        ExitMotorMode(&h2_can); can2.write(h2_can);
        ExitMotorMode(&k1_can); can1.write(k1_can);
        ExitMotorMode(&k2_can); can2.write(k2_can);
        enabled = 0;
      }
#endif
    }

    // 3) 有新的 SPI 命令帧：原子搬运 shadow -> command，再刷新 TX/CAN。
    if (g_cmd_pending) {
      __disable_irq();
      spi_command = spi_rx_shadow;
      g_cmd_pending = 0;
      __enable_irq();

      control_and_build_tx();
      PackAll();
      WriteAll();
    }

    // 4) 定期打印 SPI 帧统计，便于调试链路质量。
    uint32_t now = t.read_ms();
    if (now - last_ms >= 500) {
      uint32_t df = g_spi_frames - last_frames;
      uint32_t db = g_cmd_bad    - last_bad;
      last_frames = g_spi_frames;
      last_bad    = g_cmd_bad;
      last_ms     = now;

      pc.printf("[SPI] frames(+%lu), bad(+%lu), total=%lu/%lu, last_len=%d, estop=%d, enabled=%d\n\r",
                (unsigned long)df, (unsigned long)db,
                (unsigned long)g_spi_frames, (unsigned long)g_cmd_bad,
                g_last_len, estop.read(), enabled);
    }
  }
}


