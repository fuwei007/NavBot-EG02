/*!
 * @file HardwareBridge.cpp
 * @brief Interface between robot code and robot hardware
 *
 * This class initializes the hardware of both robots and allows the robot
 * controller to access it
 */
#ifdef linux 

#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <cstdlib>
#include "Configuration.h"

#include "HardwareBridge.h"
//#include "rt/rt_rc_interface.h"
#include "rt/rt_sbus.h"
#include "rt/rt_spi.h"
#include "rt/rt_vectornav.h"
#include "rt/rt_ethercat.h"
#include "main_helper.h"
#include "Utilities/Utilities_print.h"
#include "Utilities/Log.h"

/*!
 * If an error occurs during initialization, before motors are enabled, print
 * error and exit.
 * @param reason Error message string
 * @param printErrno If true, also print C errno
 */
void HardwareBridge::initError(const char* reason, bool printErrno) {
  LOG_CRITICAL("FAILED TO INITIALIZE HARDWARE: {}", reason);

  if (printErrno) {
    LOG_CRITICAL("Error: {}", strerror(errno));
  }

  exit(-1);
}

/*!
 * All hardware initialization steps that are common between Cheetah 3 and Mini Cheetah
 */
void HardwareBridge::initCommon() {
  LOG_INFO("[HardwareBridge] Init stack");
  prefaultStack();
  LOG_INFO("[HardwareBridge] Init scheduler");
  setupScheduler();
  if (!_interfaceLCM.good()) {
    initError("_interfaceLCM failed to initialize\n", false);
  }

  LOG_INFO("[HardwareBridge] Subscribe LCM");
  _interfaceLCM.subscribe("interface", &HardwareBridge::handleGamepadLCM, this);
  _interfaceLCM.subscribe("interface_request",
                          &HardwareBridge::handleControlParameter, this);

  LOG_INFO("[HardwareBridge] Start interface LCM handler");
  _interfaceLcmThread = std::thread(&HardwareBridge::handleInterfaceLCM, this);
}

/*!
 * Run interface LCM
 */
void HardwareBridge::handleInterfaceLCM() {
  while (!_interfaceLcmQuit) _interfaceLCM.handle();
}

/*!
 * Writes to a 16 KB buffer on the stack. If we are using 4K pages for our
 * stack, this will make sure that we won't have a page fault when the stack
 * grows.  Also mlock's all pages associated with the current process, which
 * prevents the cheetah software from being swapped out.  If we do run out of
 * memory, the robot program will be killed by the OOM process killer (and
 * leaves a log) instead of just becoming unresponsive.
 */
void HardwareBridge::prefaultStack() {
  LOG_INFO("[Init] Prefault stack...");
  volatile char stack[MAX_STACK_SIZE];
  memset(const_cast<char*>(stack), 0, MAX_STACK_SIZE);
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    initError(
        "mlockall failed.  This is likely because you didn't run robot as "
        "root.\n",
        true);
  }
}

/*!
 * Configures the scheduler for real time priority
 */
void HardwareBridge::setupScheduler() {
  LOG_INFO("[Init] Setup RT Scheduler...");
  struct sched_param params;
  params.sched_priority = TASK_PRIORITY;
  if (sched_setscheduler(0, SCHED_FIFO, &params) == -1) {
    initError("sched_setscheduler failed.\n", true);
  }
}

/*!
 * LCM Handler for gamepad message
 */
void HardwareBridge::handleGamepadLCM(const lcm::ReceiveBuffer* rbuf,
                                      const std::string& chan,
                                      const gamepad_lcmt* msg) {
  (void)rbuf;
  (void)chan;
  _gamepadCommand.set(msg);
}

/*!
 * LCM Handler for control parameters
 */
void HardwareBridge::handleControlParameter(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const control_parameter_request_lcmt* msg) {
  (void)rbuf;
  (void)chan;
  if (msg->requestNumber <= _parameter_response_lcmt.requestNumber) {
    // nothing to do!
    LOG_WARN(
        "[HardwareBridge] Warning: the interface has run a ControlParameter "
        "iteration, but there is no new request!");
    // return;
  }

  // sanity check
  s64 nRequests = msg->requestNumber - _parameter_response_lcmt.requestNumber;
  if (nRequests != 1) {
    LOG_ERROR("[ERROR] Hardware bridge: we've missed {} requests",
           nRequests - 1);
  }

  switch (msg->requestKind) {
    case (s8)ControlParameterRequestKind::SET_USER_PARAM_BY_NAME: {
      if(!_userControlParameters) {
        LOG_WARN("[Warning] Got user param {}, but not using user parameters!",
               (char*)msg->name);
      } else {
        std::string name((char*)msg->name);
        ControlParameter& param = _userControlParameters->collection.lookup(name);

        // type check
        if ((s8)param._kind != msg->parameterKind) {
          throw std::runtime_error(
              "type mismatch for parameter " + name + ", robot thinks it is " +
              controlParameterValueKindToString(param._kind) +
              " but received a command to set it to " +
              controlParameterValueKindToString(
                  (ControlParameterValueKind)msg->parameterKind));
        }

        // do the actual set
        ControlParameterValue v;
        memcpy(&v, msg->value, sizeof(v));
        param.set(v, (ControlParameterValueKind)msg->parameterKind);

        // respond:
        _parameter_response_lcmt.requestNumber =
            msg->requestNumber;  // acknowledge that the set has happened
        _parameter_response_lcmt.parameterKind =
            msg->parameterKind;  // just for debugging print statements
        memcpy(_parameter_response_lcmt.value, msg->value, 64);
        //_parameter_response_lcmt.value = _parameter_request_lcmt.value; // just
        //for debugging print statements
        strcpy((char*)_parameter_response_lcmt.name,
               name.c_str());  // just for debugging print statements
        _parameter_response_lcmt.requestKind = msg->requestKind;

        LOG_INFO("[User Control Parameter] set {} to {}", name,
               controlParameterValueToString(
                   v, (ControlParameterValueKind)msg->parameterKind));
      }
    } break;

    case (s8)ControlParameterRequestKind::SET_ROBOT_PARAM_BY_NAME: {
      std::string name((char*)msg->name);
      ControlParameter& param = _robotParams.collection.lookup(name);
      const bool ignoreCheater =
          (!gMasterConfig.simulated) && (name == "cheater_mode");

      // type check
      if ((s8)param._kind != msg->parameterKind) {
        throw std::runtime_error(
            "type mismatch for parameter " + name + ", robot thinks it is " +
            controlParameterValueKindToString(param._kind) +
            " but received a command to set it to " +
            controlParameterValueKindToString(
                (ControlParameterValueKind)msg->parameterKind));
      }

      // do the actual set
      ControlParameterValue v;
      if (ignoreCheater) {
        static bool warned = false;
        if (!warned) {
          LOG_WARN(
              "[HardwareBridge] Ignoring cheater_mode update while running on "
              "hardware");
          warned = true;
        }
        v = param.get(param._kind);
      } else {
        memcpy(&v, msg->value, sizeof(v));
        param.set(v, (ControlParameterValueKind)msg->parameterKind);
      }

      // respond:
      _parameter_response_lcmt.requestNumber =
          msg->requestNumber;  // acknowledge that the set has happened
      _parameter_response_lcmt.parameterKind =
          msg->parameterKind;  // just for debugging print statements
      memset(_parameter_response_lcmt.value, 0,
             sizeof(_parameter_response_lcmt.value));
      memcpy(_parameter_response_lcmt.value, &v, sizeof(v));
      //_parameter_response_lcmt.value = _parameter_request_lcmt.value; // just
      //for debugging print statements
      strcpy((char*)_parameter_response_lcmt.name,
             name.c_str());  // just for debugging print statements
      _parameter_response_lcmt.requestKind = msg->requestKind;

      if (!ignoreCheater) {
        LOG_INFO("[Robot Control Parameter] set {} to {}", name,
               controlParameterValueToString(
                   v, (ControlParameterValueKind)msg->parameterKind));
      }

    } break;

    default: {
      throw std::runtime_error("parameter type unsupported");
    }
    break;
  }
  _interfaceLCM.publish("interface_response", &_parameter_response_lcmt);
}


MiniCheetahHardwareBridge::MiniCheetahHardwareBridge(RobotController* robot_ctrl, bool load_parameters_from_file)
    : HardwareBridge(robot_ctrl), _spiLcm(getLcmUrl(255)), _imuLcm(getLcmUrl(255)) {
  _load_parameters_from_file = load_parameters_from_file;
}

/*!
 * Main method for Mini Cheetah hardware
 */
void MiniCheetahHardwareBridge::run() {
  initCommon();
  initHardware();

  if(_load_parameters_from_file) {
    LOG_INFO("[Hardware Bridge] Loading parameters from file...");

    try {
      _robotParams.initializeFromYamlFile(THIS_COM "config/mini-cheetah-defaults.yaml");
    } catch(std::exception& e) {
      LOG_ERROR("Failed to initialize robot parameters from yaml file: {}", e.what());
      exit(1);
    }

    if(!_robotParams.isFullyInitialized()) {
      LOG_ERROR("Failed to initialize all robot parameters");
      exit(1);
    }

    LOG_INFO("Loaded robot parameters");

    if(_userControlParameters) {
      std::string userParamPath = THIS_COM "config/mc-mit-ctrl-user-parameters.yaml";
      bool hasTauFF = false;
      bool hasKdJointVec = false;
      try {
        (void)_userControlParameters->collection.lookup("tau_ff");
        hasTauFF = true;
      } catch (std::exception&) {
        hasTauFF = false;
      }
      try {
        (void)_userControlParameters->collection.lookup("Kd_joint");
        hasKdJointVec = true;
      } catch (std::exception&) {
        hasKdJointVec = false;
      }
      if (hasTauFF && !hasKdJointVec) {
        userParamPath = THIS_COM "config/mc-jpos-user-parameters.yaml";
      }
      try {
        _userControlParameters->initializeFromYamlFile(userParamPath);
      } catch(std::exception& e) {
        LOG_ERROR("Failed to initialize user parameters from yaml file: {}", e.what());
        exit(1);
      }

      if(!_userControlParameters->isFullyInitialized()) {
        LOG_ERROR("Failed to initialize all user parameters");
        exit(1);
      }

      LOG_INFO("Loaded user parameters");
    } else {
      LOG_INFO("Did not load user parameters because there aren't any");
    }
  } else {
    LOG_INFO("[Hardware Bridge] Loading parameters over LCM...");
    while (!_robotParams.isFullyInitialized()) {
      LOG_INFO("[Hardware Bridge] Waiting for robot parameters...");
      usleep(1000000);
    }

    if(_userControlParameters) {
      while (!_userControlParameters->isFullyInitialized()) {
        LOG_INFO("[Hardware Bridge] Waiting for user parameters...");
        usleep(1000000);
      }
    }
  }



  LOG_INFO("[Hardware Bridge] Got all parameters, starting up!");
  _robotParams.use_rc = 0;
  LOG_INFO("[Hardware Bridge] use_rc overridden to 0 (gamepad mode)");
  if (_robotParams.cheater_mode != 0) {
    _robotParams.cheater_mode = 0;
    LOG_WARN("[Hardware Bridge] cheater_mode overridden to 0 (cheater state unavailable on hardware)");
  }

  _robotRunner =
      new RobotRunner(_controller, &taskManager, _robotParams.controller_dt, "robot-control");

  _robotRunner->driverCommand = &_gamepadCommand;
  _robotRunner->spiData = &_spiData;
  _robotRunner->spiCommand = &_spiCommand;
  _robotRunner->robotType = RobotType::MINI_CHEETAH;
  _robotRunner->vectorNavData = &_vectorNavData;
  _robotRunner->controlParameters = &_robotParams;
  _robotRunner->visualizationData = &_visualizationData;
  _robotRunner->cheetahMainVisualization = &_mainCheetahVisualization;

  _firstRun = false;

  // init control thread

  statusTask.start();

  // spi Task start
  PeriodicMemberFunction<MiniCheetahHardwareBridge> spiTask(
      &taskManager, .002, "spi", &MiniCheetahHardwareBridge::runSpi, this);
  spiTask.start();

  // IMU
  if (_imuInit)
    _imuThread = std::thread(&MiniCheetahHardwareBridge::runImu, this);

  // robot controller start
  _robotRunner->start();

  // visualization start
  PeriodicMemberFunction<MiniCheetahHardwareBridge> visualizationLCMTask(
      &taskManager, .0167, "lcm-vis",
      &MiniCheetahHardwareBridge::publishVisualizationLCM, this);
  visualizationLCMTask.start();

  _gamepadThread = std::thread(&MiniCheetahHardwareBridge::runGamepad, this);
  if (!_gamepadInit) {
    LOG_WARN("[Gamepad] Warning: no joystick detected, commands will remain zero until it becomes available.");
  }

  // IMU logger
  PeriodicMemberFunction<MiniCheetahHardwareBridge> imuLogger(
      &taskManager, .001, "imu-logger", &MiniCheetahHardwareBridge::logImu, this);
  imuLogger.start();

  for (;;) {
    usleep(1000000);
    // printf("joy %f\n", _robotRunner->driverCommand->leftStickAnalog[0]);
  }
}

/*!
 * Receive RC with SBUS
 */
void HardwareBridge::run_sbus() {
  if (_port > 0) {
    int x = receive_sbus(_port);
    if (x) {
      sbus_packet_complete();
    }
  }
}

void MiniCheetahHardwareBridge::runImu() {
  while (true) {
    _wheeltecImu.run();
    if (_imuInit && _wheeltecImu.isInitialized()) {
      _wheeltecImu.copyToVectorNav(&_vectorNavData);
    } else {
      usleep(1000);
    }
  }
}

void MiniCheetahHardwareBridge::runGamepad() {
  GamepadCommand localCommand;
  while (true) {
    if (!_linuxGamepad.isOpen()) {
      _gamepadInit = false;
      _gamepadCommand.zero();
      if (!_gamepadDevice.empty()) {
        if (_linuxGamepad.openDevice(_gamepadDevice)) {
          LOG_INFO("[Gamepad] Reconnected to {}", _gamepadDevice);
          _gamepadInit = true;
        }
      }
      usleep(200000);
      continue;
    }

    if (_linuxGamepad.poll(localCommand)) {
      _gamepadCommand = localCommand;
    } else {
      usleep(5000);
    }
  }
}

void MiniCheetahHardwareBridge::logImu() {
  if (!_imuInit || !_wheeltecImu.isInitialized())
    return;

  _wheeltecImu.updateLCM(&_imuData);
  _imuLcm.publish("microstrain", &_imuData);
}

/*!
 * Initialize Mini Cheetah specific hardware
 */
void MiniCheetahHardwareBridge::initHardware() {
  _vectorNavData.quat << 1, 0, 0, 0;
  _vectorNavData.accelerometer.setZero();
  _vectorNavData.gyro.setZero();

  init_spi();

  const char* gamepadEnv = std::getenv("F710_DEVICE");
  _gamepadDevice = gamepadEnv ? gamepadEnv : std::string("/dev/input/js0");
  _gamepadInit = _linuxGamepad.openDevice(_gamepadDevice);
  if (_gamepadInit) {
    LOG_INFO("[Gamepad] Using {} for Logitech F710 input", _gamepadDevice);
  } else {
    LOG_ERROR(
                 "[Gamepad] Failed to open {}. Gamepad commands will stay zero until it becomes available.",
                 _gamepadDevice);
  }

  const char* portEnv = std::getenv("H30_IMU_PORT");
  std::string device = portEnv ? portEnv : "/dev/ttyACM0";

  int baud = 460800;
  if (const char* baudEnv = std::getenv("H30_IMU_BAUD")) {
    int parsed = std::atoi(baudEnv);
    if (parsed > 0) {
      baud = parsed;
    }
  }

  _imuInit = _wheeltecImu.tryInit(device, baud);
  if (!_imuInit) {
    LOG_ERROR(
                 "[WheeltecImu] Failed to initialize on {} @ {} baud",
                 device, baud);
  } else {
    LOG_INFO("[WheeltecImu] Streaming from {} @ {} baud", device, baud);
  }
}

/*!
 * Run Mini Cheetah SPI
 */
void MiniCheetahHardwareBridge::runSpi() {
  spi_command_t* cmd = get_spi_command();
  spi_data_t* data = get_spi_data();

  memcpy(cmd, &_spiCommand, sizeof(spi_command_t));
  spi_driver_run();
  memcpy(&_spiData, data, sizeof(spi_data_t));

  _spiLcm.publish("spi_data", data);
  _spiLcm.publish("spi_command", cmd);
}

void Cheetah3HardwareBridge::initHardware() {
  _vectorNavData.quat << 1, 0, 0, 0;
  LOG_INFO("[Cheetah 3 Hardware] Init vectornav");
  if (!init_vectornav(&_vectorNavData)) {
    LOG_ERROR("Vectornav failed to initialize");
    LOG_CRITICAL("****************\n**  WARNING!  **\n****************\n  IMU DISABLED  \n****************");
    //initError("failed to initialize vectornav!\n", false);
  }
}



void Cheetah3HardwareBridge::runEcat() {
  rt_ethercat_set_command(_tiBoardCommand);
  rt_ethercat_run();
  rt_ethercat_get_data(_tiBoardData);

  publishEcatLCM();
}

void Cheetah3HardwareBridge::publishEcatLCM() {
  for(int leg = 0; leg < 4; leg++) {
    ecatCmdLcm.x_des[leg] = _tiBoardCommand[leg].position_des[0];
    ecatCmdLcm.y_des[leg] = _tiBoardCommand[leg].position_des[1];
    ecatCmdLcm.z_des[leg] = _tiBoardCommand[leg].position_des[2];
    ecatCmdLcm.dx_des[leg] = _tiBoardCommand[leg].velocity_des[0];
    ecatCmdLcm.dy_des[leg] = _tiBoardCommand[leg].velocity_des[1];
    ecatCmdLcm.dz_des[leg] = _tiBoardCommand[leg].velocity_des[2];
    ecatCmdLcm.kpx[leg] = _tiBoardCommand[leg].kp[0];
    ecatCmdLcm.kpy[leg] = _tiBoardCommand[leg].kp[1];
    ecatCmdLcm.kpz[leg] = _tiBoardCommand[leg].kp[2];
    ecatCmdLcm.kdx[leg] = _tiBoardCommand[leg].kd[0];
    ecatCmdLcm.kdy[leg] = _tiBoardCommand[leg].kd[1];
    ecatCmdLcm.kdz[leg] = _tiBoardCommand[leg].kd[2];
    ecatCmdLcm.enable[leg] = _tiBoardCommand[leg].enable;
    ecatCmdLcm.zero_joints[leg] = _tiBoardCommand[leg].zero;
    ecatCmdLcm.fx_ff[leg] = _tiBoardCommand[leg].force_ff[0];
    ecatCmdLcm.fy_ff[leg] = _tiBoardCommand[leg].force_ff[1];
    ecatCmdLcm.fz_ff[leg] = _tiBoardCommand[leg].force_ff[2];
    ecatCmdLcm.tau_abad_ff[leg] = _tiBoardCommand[leg].tau_ff[0];
    ecatCmdLcm.tau_hip_ff[leg] = _tiBoardCommand[leg].tau_ff[1];
    ecatCmdLcm.tau_knee_ff[leg] = _tiBoardCommand[leg].tau_ff[2];
    ecatCmdLcm.q_des_abad[leg] = _tiBoardCommand[leg].q_des[0];
    ecatCmdLcm.q_des_hip[leg] = _tiBoardCommand[leg].q_des[1];
    ecatCmdLcm.q_des_knee[leg] = _tiBoardCommand[leg].q_des[2];
    ecatCmdLcm.qd_des_abad[leg] = _tiBoardCommand[leg].qd_des[0];
    ecatCmdLcm.qd_des_hip[leg] = _tiBoardCommand[leg].qd_des[1];
    ecatCmdLcm.qd_des_knee[leg] = _tiBoardCommand[leg].qd_des[2];
    ecatCmdLcm.kp_joint_abad[leg] = _tiBoardCommand[leg].kp_joint[0];
    ecatCmdLcm.kp_joint_hip[leg] = _tiBoardCommand[leg].kp_joint[1];
    ecatCmdLcm.kp_joint_knee[leg] = _tiBoardCommand[leg].kp_joint[2];
    ecatCmdLcm.kd_joint_abad[leg] = _tiBoardCommand[leg].kd_joint[0];
    ecatCmdLcm.kd_joint_hip[leg] = _tiBoardCommand[leg].kd_joint[1];
    ecatCmdLcm.kd_joint_knee[leg] = _tiBoardCommand[leg].kd_joint[2];
    ecatCmdLcm.max_torque[leg] = _tiBoardCommand[leg].max_torque;
  }

  for(int leg = 0; leg < 4; leg++) {
    ecatDataLcm.x[leg] = _tiBoardData[leg].position[0];
    ecatDataLcm.y[leg] = _tiBoardData[leg].position[1];
    ecatDataLcm.z[leg] = _tiBoardData[leg].position[2];
    ecatDataLcm.dx[leg] = _tiBoardData[leg].velocity[0];
    ecatDataLcm.dy[leg] = _tiBoardData[leg].velocity[1];
    ecatDataLcm.dz[leg] = _tiBoardData[leg].velocity[2];
    ecatDataLcm.fx[leg] = _tiBoardData[leg].force[0];
    ecatDataLcm.fy[leg] = _tiBoardData[leg].force[1];
    ecatDataLcm.fz[leg] = _tiBoardData[leg].force[2];
    ecatDataLcm.q_abad[leg] = _tiBoardData[leg].q[0];
    ecatDataLcm.q_hip[leg] = _tiBoardData[leg].q[1];
    ecatDataLcm.q_knee[leg] = _tiBoardData[leg].q[2];
    ecatDataLcm.dq_abad[leg] = _tiBoardData[leg].dq[0];
    ecatDataLcm.dq_hip[leg] = _tiBoardData[leg].dq[1];
    ecatDataLcm.dq_knee[leg] = _tiBoardData[leg].dq[2];
    ecatDataLcm.tau_abad[leg] = _tiBoardData[leg].tau[0];
    ecatDataLcm.tau_hip[leg] = _tiBoardData[leg].tau[1];
    ecatDataLcm.tau_knee[leg] = _tiBoardData[leg].tau[2];
    ecatDataLcm.tau_des_abad[leg] = _tiBoardData[leg].tau_des[0];
    ecatDataLcm.tau_des_hip[leg] = _tiBoardData[leg].tau_des[1];
    ecatDataLcm.tau_des_knee[leg] = _tiBoardData[leg].tau_des[2];
    ecatDataLcm.loop_count_ti[leg] = _tiBoardData[leg].loop_count_ti;
    ecatDataLcm.ethercat_count_ti[leg] = _tiBoardData[leg].ethercat_count_ti;
    ecatDataLcm.microtime_ti[leg] = _tiBoardData[leg].microtime_ti;
  }

  _ecatLCM.publish("ecat_cmd", &ecatCmdLcm);
  _ecatLCM.publish("ecat_data", &ecatDataLcm);
}

/*!
 * Send LCM visualization data
 */
void HardwareBridge::publishVisualizationLCM() {
  cheetah_visualization_lcmt visualization_data;
  for (int i = 0; i < 3; i++) {
    visualization_data.x[i] = _mainCheetahVisualization.p[i];
  }

  for (int i = 0; i < 4; i++) {
    visualization_data.quat[i] = _mainCheetahVisualization.quat[i];
    visualization_data.rgba[i] = _mainCheetahVisualization.color[i];
  }

  for (int i = 0; i < 12; i++) {
    visualization_data.q[i] = _mainCheetahVisualization.q[i];
  }

  _visualizationLCM.publish("main_cheetah_visualization", &visualization_data);
}

Cheetah3HardwareBridge::Cheetah3HardwareBridge(RobotController *rc) : HardwareBridge(rc),  _ecatLCM(getLcmUrl(255)) {

}

void Cheetah3HardwareBridge::run() {
  initCommon();
  initHardware();

  LOG_INFO("[Hardware Bridge] Loading parameters over LCM...");
  while (!_robotParams.isFullyInitialized()) {
    LOG_INFO("[Hardware Bridge] Waiting for robot parameters...");
    usleep(1000000);
  }

  if(_userControlParameters) {
    while (!_userControlParameters->isFullyInitialized()) {
      LOG_INFO("[Hardware Bridge] Waiting for user parameters...");
      usleep(1000000);
    }
  }

  LOG_INFO("[Hardware Bridge] Got all parameters, starting up!");

  _robotRunner =
      new RobotRunner(_controller, &taskManager, _robotParams.controller_dt, "robot-control");

  _robotRunner->driverCommand = &_gamepadCommand;
  _robotRunner->tiBoardData = _tiBoardData;
  _robotRunner->tiBoardCommand = _tiBoardCommand;
  _robotRunner->robotType = RobotType::CHEETAH_3;
  _robotRunner->controlParameters = &_robotParams;
  _robotRunner->visualizationData = &_visualizationData;
  _robotRunner->cheetahMainVisualization = &_mainCheetahVisualization;
  _robotRunner->vectorNavData = &_vectorNavData;

  _robotRunner->init();
  _firstRun = false;

  // init control thread

  statusTask.start();

  rt_ethercat_init();
  // Ecat Task start
  PeriodicMemberFunction<Cheetah3HardwareBridge> ecatTask(
      &taskManager, .001, "ecat", &Cheetah3HardwareBridge::runEcat, this);
  ecatTask.start();

  // robot controller start
  _robotRunner->start();

  // visualization start
  PeriodicMemberFunction<Cheetah3HardwareBridge> visualizationLCMTask(
      &taskManager, .0167, "lcm-vis",
      &MiniCheetahHardwareBridge::publishVisualizationLCM, this);
  visualizationLCMTask.start();

  // rc controller disabled for now
//  _port = init_sbus(false);  // Not Simulation
//  PeriodicMemberFunction<HardwareBridge> sbusTask(
//      &taskManager, .005, "rc_controller", &HardwareBridge::run_sbus, this);
//  sbusTask.start();


  for (;;) {
    usleep(100000);
    taskManager.printStatus();
    // printf("joy %f\n", _robotRunner->driverCommand->leftStickAnalog[0]);
  }
}

#endif
