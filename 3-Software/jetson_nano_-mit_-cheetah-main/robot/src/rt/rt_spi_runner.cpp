#ifdef __linux__

#include "rt/rt_spi_runner.h"
#include "rt/rt_spi.h"

#include <atomic>
#include <cstdio>
#include <cstring>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <unistd.h>

namespace {

constexpr int kDefaultHz = 1000;
constexpr int kDefaultPriority = 60;
constexpr int kDefaultCpu = -1;

pthread_t g_thr{};
std::atomic<bool> g_run{false};
std::atomic<uint64_t> g_iterations{0};
std::atomic<uint64_t> g_overruns{0};

void configure_thread(int priority, int cpu){
  struct sched_param sp{};
  sp.sched_priority = priority;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0){
    std::perror("[rt_spi_runner] pthread_setschedparam");
  }
  if (cpu >= 0){
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);
    if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0){
      std::perror("[rt_spi_runner] pthread_setaffinity_np");
    }
  }
}

inline void normalize_timespec(struct timespec& ts){
  while (ts.tv_nsec >= 1000000000L){
    ts.tv_nsec -= 1000000000L;
    ++ts.tv_sec;
  }
}

void* runner(void* arg){
  rt_spi_runner_opts_t opts{};
  if (arg){
    opts = *static_cast<rt_spi_runner_opts_t*>(arg);
    delete static_cast<rt_spi_runner_opts_t*>(arg);
  }else{
    opts.hz = kDefaultHz;
    opts.rt_priority = kDefaultPriority;
    opts.cpu = kDefaultCpu;
  }
  if (opts.hz <= 0) opts.hz = kDefaultHz;
  if (opts.rt_priority <= 0) opts.rt_priority = kDefaultPriority;
  if (opts.cpu < -1) opts.cpu = kDefaultCpu;

  configure_thread(opts.rt_priority, opts.cpu);

  const long ns_per_tick = 1000000000L / opts.hz;
  struct timespec next_wake{};
  clock_gettime(CLOCK_MONOTONIC, &next_wake);

  g_iterations.store(0, std::memory_order_relaxed);
  g_overruns.store(0, std::memory_order_relaxed);

  while (g_run.load(std::memory_order_relaxed)){
    next_wake.tv_nsec += ns_per_tick;
    normalize_timespec(next_wake);

    spi_driver_run();
    g_iterations.fetch_add(1, std::memory_order_relaxed);

    const int ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wake, nullptr);
    if (ret != 0){
      g_overruns.fetch_add(1, std::memory_order_relaxed);
      clock_gettime(CLOCK_MONOTONIC, &next_wake);
    }
  }

  return nullptr;
}

} // namespace

int rt_spi_runner_start(const rt_spi_runner_opts_t* opts){
  bool expected = false;
  if (!g_run.compare_exchange_strong(expected, true, std::memory_order_acq_rel)){
    return 0;  // already running
  }

  init_spi();

  auto* runner_opts = new rt_spi_runner_opts_t;
  if (opts){
    *runner_opts = *opts;
  }else{
    runner_opts->hz = kDefaultHz;
    runner_opts->rt_priority = kDefaultPriority;
    runner_opts->cpu = kDefaultCpu;
  }

  const int err = pthread_create(&g_thr, nullptr, runner, runner_opts);
  if (err != 0){
    std::fprintf(stderr, "[rt_spi_runner] pthread_create: %s\n", std::strerror(err));
    delete runner_opts;
    g_run.store(false, std::memory_order_release);
    return -1;
  }

  usleep(1000);
  return 0;
}

void rt_spi_runner_stop(void){
  if (!g_run.exchange(false, std::memory_order_acq_rel)){
    return;
  }
  pthread_join(g_thr, nullptr);
}

rt_spi_runner_stats_t rt_spi_runner_get_stats(void){
  rt_spi_runner_stats_t stats{};
  stats.iterations = g_iterations.load(std::memory_order_relaxed);
  stats.overruns = g_overruns.load(std::memory_order_relaxed);
  return stats;
}

#endif  // __linux__
