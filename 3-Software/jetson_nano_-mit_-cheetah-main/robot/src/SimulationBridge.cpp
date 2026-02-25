/*! @file SimulationBridge.cpp
 *  @brief  The SimulationBridge runs a RobotController and connects it to a
 * Simulator, using shared memory. It is the simulation version of the
 * HardwareBridge.
 */

#include "SimulationBridge.h"
#include <cstdlib>
#include <unistd.h>

#include "Utilities/SegfaultHandler.h"
#include "Controllers/LegController.h"

/*!
 * Connect to a simulation
 */
void SimulationBridge::run() {
  // init shared memory:
  _sharedMemory.attach(DEVELOPMENT_SIMULATOR_SHARED_MEMORY_NAME);
  _sharedMemory().init();

  install_segfault_handler(_sharedMemory().robotToSim.errorMessage);

  // init Quadruped Controller

  try {
    printf("[Simulation Driver] Starting main loop...\n");
    bool firstRun = true;
    for (;;) {
      // wait for our turn to access the shared memory
      // on the first loop, this gives the simulator a chance to put stuff in
      // shared memory before we start
      _sharedMemory().waitForSimulator();

      if (firstRun) {
        firstRun = false;
        // check that the robot type is correct:
        if (_robot != _sharedMemory().simToRobot.robotType) {
          printf(
            "simulator and simulatorDriver don't agree on which robot we are "
            "simulating (robot %d, sim %d)\n",
            (int)_robot, (int)_sharedMemory().simToRobot.robotType);
          throw std::runtime_error("robot mismatch!");
        }
      }

      // the simulator tells us which mode to run in
      _simMode = _sharedMemory().simToRobot.mode;
      switch (_simMode) {
        case SimulatorMode::RUN_CONTROL_PARAMETERS:  // there is a new control
          // parameter request
          handleControlParameters();
          break;
        case SimulatorMode::RUN_CONTROLLER:  // the simulator is ready for the
          // next robot controller run
          _iterations++;
          runRobotControl();
          break;
        case SimulatorMode::DO_NOTHING:  // the simulator is just checking to see
          // if we are alive yet
          break;
        case SimulatorMode::EXIT:  // the simulator is done with us
          printf("[Simulation Driver] Transitioned to exit mode\n");
          return;
          break;
        default:
          throw std::runtime_error("unknown simulator mode");
      }

      // tell the simulator we are done
      _sharedMemory().robotIsDone();
    }
  } catch (std::exception& e) {
    snprintf(_sharedMemory().robotToSim.errorMessage, sizeof(_sharedMemory().robotToSim.errorMessage), "%s", e.what());
    throw e;
  }

}

/*!
 * This function handles a a control parameter message from the simulator
 */
void SimulationBridge::handleControlParameters() {
  ControlParameterRequest& request =
      _sharedMemory().simToRobot.controlParameterRequest;
  ControlParameterResponse& response =
      _sharedMemory().robotToSim.controlParameterResponse;
  if (request.requestNumber <= response.requestNumber) {
    // nothing to do!
    printf(
        "[SimulationBridge] Warning: the simulator has run a ControlParameter "
        "iteration, but there is no new request!\n");
    return;
  }

  // sanity check
  u64 nRequests = request.requestNumber - response.requestNumber;
  assert(nRequests == 1);

  response.nParameters = _robotParams.collection._map
                             .size();  // todo don't do this every single time?

  switch (request.requestKind) {
    case ControlParameterRequestKind::SET_ROBOT_PARAM_BY_NAME: {
      std::string name(request.name);
      ControlParameter& param = _robotParams.collection.lookup(name);

      // type check
      if (param._kind != request.parameterKind) {
        throw std::runtime_error(
            "type mismatch for parameter " + name + ", robot thinks it is " +
            controlParameterValueKindToString(param._kind) +
            " but received a command to set it to " +
            controlParameterValueKindToString(request.parameterKind));
      }

      // do the actual set
      param.set(request.value, request.parameterKind);

      // respond:
      response.requestNumber =
          request.requestNumber;  // acknowledge that the set has happened
      response.parameterKind =
          request.parameterKind;       // just for debugging print statements
      response.value = request.value;  // just for debugging print statements
      strcpy(response.name,
             name.c_str());  // just for debugging print statements
      response.requestKind = request.requestKind;

      printf("%s\n", response.toString().c_str());

    } break;

    case ControlParameterRequestKind::SET_USER_PARAM_BY_NAME: {
      std::string name(request.name);
      if(!_userParams) {
        printf("[Simulation Bridge] Warning: tried to set user parameter, but the robot does not have any!\n");
      } else {
        ControlParameter& param = _userParams->collection.lookup(name);

        // type check
        if (param._kind != request.parameterKind) {
          throw std::runtime_error(
              "type mismatch for parameter " + name + ", robot thinks it is " +
              controlParameterValueKindToString(param._kind) +
              " but received a command to set it to " +
              controlParameterValueKindToString(request.parameterKind));
        }

        // do the actual set
        param.set(request.value, request.parameterKind);
      }

      // respond:
      response.requestNumber =
          request.requestNumber;  // acknowledge that the set has happened
      response.parameterKind =
          request.parameterKind;       // just for debugging print statements
      response.value = request.value;  // just for debugging print statements
      strcpy(response.name,
             name.c_str());  // just for debugging print statements
      response.requestKind = request.requestKind;

      printf("%s\n", response.toString().c_str());

    } break;

    case ControlParameterRequestKind::GET_ROBOT_PARAM_BY_NAME: {
      std::string name(request.name);
      ControlParameter& param = _robotParams.collection.lookup(name);

      // type check
      if (param._kind != request.parameterKind) {
        throw std::runtime_error(
            "type mismatch for parameter " + name + ", robot thinks it is " +
            controlParameterValueKindToString(param._kind) +
            " but received a command to set it to " +
            controlParameterValueKindToString(request.parameterKind));
      }

      // respond
      response.value = param.get(request.parameterKind);
      response.requestNumber = request.requestNumber;  // acknowledge
      response.parameterKind =
          request.parameterKind;  // just for debugging print statements
      strcpy(response.name,
             name.c_str());  // just for debugging print statements
      response.requestKind =
          request.requestKind;  // just for debugging print statements

      printf("%s\n", response.toString().c_str());
    } break;
    default:
      throw std::runtime_error("unhandled get/set");
  }
}

/*!
 * Run the robot controller
 */
void SimulationBridge::runRobotControl() {
  if (_firstControllerRun) {
    printf("[Simulator Driver] First run of robot controller...\n");
    if (_robotParams.isFullyInitialized()) {
      printf("\tAll %ld control parameters are initialized\n",
             _robotParams.collection._map.size());
    } else {
      printf(
          "\tbut not all control parameters were initialized. Missing:\n%s\n",
          _robotParams.generateUnitializedList().c_str());
      throw std::runtime_error(
          "not all parameters initialized when going into RUN_CONTROLLER");
    }

    auto* userControlParameters = _robotRunner->_robot_ctrl->getUserControlParameters();
    if(userControlParameters) {
      if (userControlParameters->isFullyInitialized()) {
        printf("\tAll %ld user parameters are initialized\n",
               userControlParameters->collection._map.size());
        _simMode = SimulatorMode::RUN_CONTROLLER;
      } else {
        printf(
            "\tbut not all control parameters were initialized. Missing:\n%s\n",
            userControlParameters->generateUnitializedList().c_str());
        throw std::runtime_error(
            "not all parameters initialized when going into RUN_CONTROLLER");
      }
    } else {
      _simMode = SimulatorMode::RUN_CONTROLLER;
    }


    _gamepadCommand.zero();
    const char* gamepadEnv = std::getenv("F710_DEVICE");
    _gamepadDevice = gamepadEnv ? gamepadEnv : std::string("/dev/input/js0");
    _gamepadInit = _linuxGamepad.openDevice(_gamepadDevice);
    if (_gamepadInit) {
      printf("[Gamepad] Using %s for Logitech F710 input\n", _gamepadDevice.c_str());
    } else {
      printf("[Gamepad] Failed to open %s. Falling back to simulator inputs and retrying...\n",
             _gamepadDevice.c_str());
    }

    _gamepadThread = std::thread(&SimulationBridge::runGamepad, this);
    _gamepadThread.detach();

    _robotRunner->driverCommand = &_gamepadCommand;
    _robotRunner->spiData = &_sharedMemory().simToRobot.spiData;
    _robotRunner->tiBoardData = _sharedMemory().simToRobot.tiBoardData;
    _robotRunner->robotType = _robot;
    _robotRunner->vectorNavData = &_sharedMemory().simToRobot.vectorNav;
    _robotRunner->cheaterState = &_sharedMemory().simToRobot.cheaterState;
    _robotRunner->spiCommand = &_sharedMemory().robotToSim.spiCommand;
    _robotRunner->tiBoardCommand =
        _sharedMemory().robotToSim.tiBoardCommand;
    _robotRunner->controlParameters = &_robotParams;
    _robotRunner->visualizationData =
        &_sharedMemory().robotToSim.visualizationData;
    _robotRunner->cheetahMainVisualization =
        &_sharedMemory().robotToSim.mainCheetahVisualization;

    _robotParams.use_rc = 0;
    printf("[Simulation Bridge] use_rc overridden to 0 (gamepad mode)\n");

    _robotRunner->init();
    _firstControllerRun = false;
  }

  if (!_gamepadInit) {
    _gamepadCommand = _sharedMemory().simToRobot.gamepadCommand;
  }
  _robotRunner->run();
}

void SimulationBridge::runGamepad() {
  GamepadCommand localCommand;
  while (true) {
    if (!_linuxGamepad.isOpen()) {
      _gamepadInit = false;
      if (!_gamepadDevice.empty()) {
        if (_linuxGamepad.openDevice(_gamepadDevice)) {
          printf("[Gamepad] Reconnected to %s\n", _gamepadDevice.c_str());
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
