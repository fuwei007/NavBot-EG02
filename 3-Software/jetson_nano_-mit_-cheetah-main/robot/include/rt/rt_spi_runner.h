#pragma once

#ifdef __linux__

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct rt_spi_runner_opts {
  int hz;           // loop frequency in Hz
  int rt_priority;  // SCHED_FIFO priority (60-80 suggested)
  int cpu;          // CPU affinity (-1 for none)
} rt_spi_runner_opts_t;

int rt_spi_runner_start(const rt_spi_runner_opts_t* opts);
void rt_spi_runner_stop(void);

typedef struct rt_spi_runner_stats {
  uint64_t iterations;  // total iterations executed
  uint64_t overruns;    // cycle overruns detected
} rt_spi_runner_stats_t;

rt_spi_runner_stats_t rt_spi_runner_get_stats(void);

#ifdef __cplusplus
}
#endif

#endif  // __linux__
