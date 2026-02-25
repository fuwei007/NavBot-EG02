/*!
 * @file PeriodicTask.cpp
 * @brief Implementation of a periodic function running in a separate thread.
 * Periodic tasks have a task manager, which measure how long they take to run.
 */
#ifdef linux
 #include <sys/timerfd.h>
#endif

#include <unistd.h>
#include <cmath>

#include "Utilities/PeriodicTask.h"
#include "Utilities/Timer.h"
#include "Utilities/Utilities_print.h"
#include "Utilities/Log.h"


/*!
 * Construct a new task within a TaskManager
 * @param taskManager : Parent task manager
 * @param period : how often to run
 * @param name : name of task
 */
PeriodicTask::PeriodicTask(PeriodicTaskManager* taskManager, float period,
                           std::string name)
    : _period(period), _name(name) {
  taskManager->addTask(this);
}

/*!
 * Begin running task
 */
void PeriodicTask::start() {
  if (_running) {
    LOG_WARN("[PeriodicTask] Tried to start {} but it was already running!",
           _name);
    return;
  }
  init();
  _running = true;
  _thread = std::thread(&PeriodicTask::loopFunction, this);
}

/*!
 * Stop running task
 */
void PeriodicTask::stop() {
  if (!_running) {
    LOG_WARN("[PeriodicTask] Tried to stop {} but it wasn't running!",
           _name);
    return;
  }
  _running = false;
  LOG_INFO("[PeriodicTask] Waiting for {} to stop...", _name);
  _thread.join();
  LOG_INFO("[PeriodicTask] Done!");
  cleanup();
}

/*!
 * If max period is more than 30% over desired period, it is slow
 */
bool PeriodicTask::isSlow() {
  return _maxPeriod > _period * 1.3f || _maxRuntime > _period;
}

/*!
 * Reset max statistics
 */
void PeriodicTask::clearMax() {
  _maxPeriod = 0;
  _maxRuntime = 0;
}

/*!
 * Print the status of this task in the table format
 */
void PeriodicTask::printStatus() {
  if (!_running) return;
  if (isSlow()) {
    LOG_WARN("|{:<20}|{:6.4f}|{:6.4f}|{:6.4f}|{:6.4f}|{:6.4f}",
                 _name, _lastRuntime, _maxRuntime, _period,
                 _lastPeriodTime, _maxPeriod);
  } else {
    LOG_INFO("|{:<20}|{:6.4f}|{:6.4f}|{:6.4f}|{:6.4f}|{:6.4f}", _name,
           _lastRuntime, _maxRuntime, _period, _lastPeriodTime, _maxPeriod);
  }
}

/*!
 * Call the task in a timed loop.  Uses a timerfd
 */
void PeriodicTask::loopFunction() {
#ifdef linux
  auto timerFd = timerfd_create(CLOCK_MONOTONIC, 0);
#endif
  int seconds = (int)_period;
  int nanoseconds = (int)(1e9 * std::fmod(_period, 1.f));

  Timer t;

#ifdef linux
  itimerspec timerSpec;
  timerSpec.it_interval.tv_sec = seconds;
  timerSpec.it_value.tv_sec = seconds;
  timerSpec.it_value.tv_nsec = nanoseconds;
  timerSpec.it_interval.tv_nsec = nanoseconds;

  timerfd_settime(timerFd, 0, &timerSpec, nullptr);
#endif
  unsigned long long missed = 0;

  LOG_INFO("[PeriodicTask] Start {} ({} s, {} ns)", _name, seconds,
         nanoseconds);
  while (_running) {
    _lastPeriodTime = (float)t.getSeconds();
    t.start();
    run();
    _lastRuntime = (float)t.getSeconds();
#ifdef linux
    int m = read(timerFd, &missed, sizeof(missed));
    (void)m;
#endif
    _maxPeriod = std::max(_maxPeriod, _lastPeriodTime);
    _maxRuntime = std::max(_maxRuntime, _lastRuntime);
  }
  LOG_INFO("[PeriodicTask] {} has stopped!", _name);
}

PeriodicTaskManager::~PeriodicTaskManager() {}

/*!
 * Add a new task to a task manager
 */
void PeriodicTaskManager::addTask(PeriodicTask* task) {
  _tasks.push_back(task);
}

/*!
 * Print the status of all tasks and rest max statistics
 */
void PeriodicTaskManager::printStatus() {
  LOG_INFO("\n----------------------------TASKS----------------------------");
  LOG_INFO("|{:<20}|{:<6}|{:<6}|{:<6}|{:<6}|{:<6}", "name", "rt", "rt-max", "T-des",
         "T-act", "T-max");
  LOG_INFO("-----------------------------------------------------------");
  for (auto& task : _tasks) {
    task->printStatus();
    task->clearMax();
  }
  LOG_INFO("-------------------------------------------------------------\n");
}

/*!
 * Print only the slow tasks
 */
void PeriodicTaskManager::printStatusOfSlowTasks() {
  for (auto& task : _tasks) {
    if (task->isSlow()) {
      task->printStatus();
      task->clearMax();
    }
  }
}

/*!
 * Stop all tasks
 */
void PeriodicTaskManager::stopAll() {
  for (auto& task : _tasks) {
    task->stop();
  }
}
