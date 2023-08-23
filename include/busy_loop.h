/*!
 * \file busy_loop.h
 * \brief Header to use busy loops (simulate computation time)
 * \author Etienne Hamelin <etienne.hamelin@cea.fr> 
 * \author Alexandre Berne <alexandre.berne@cea.fr> 
 * \version 1.0
 * \date 03/08/2023
 *
 * Header to use busy loops (simulate computation time)
 * (c) CEA List - DRT/LIST/DSCIN/LCYL
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <cassert>
// Header for time measurement
#include <time.h>

#ifndef LOOPS_PER_NS
#define LOOPS_PER_NS 0.0f
#endif

/** @brief convert microseconds to nanoseconds */
#define US2NS(us)  ((us)*1000)

/** @brief convert milliseconds to nanoseconds */
#define MS2NS(ms)  ((ms)*1000*1000)

/** @brief convert seconds to nanoseconds */
#define S2NS(s)    ((s)*1000*1000*1000)

/** @brief Macro token concatenation */
#define CONCAT(a, b) a ## b
#define UNIQUENAME(prefix) CONCAT(prefix, __LINE__)

#define MIN(a,b) (((a)<(b)) ? (a) : (b))
#define MAX(a,b) (((a)>(b)) ? (a) : (b))

#define REPEAT_X10(block) { block; block; block;block; block; block; block; block; block; block; }

/** @brief Timestamp of last DEBUG statement */
struct timespec last_debug_ts;

/**
 * @brief duration = timestamp[to] - timestamp[from]
 * @param from_ts timestsamp "from"
 * @param to_ts timestamp "to"
 * @return duration in nanoseconds (/!\ might overflow if there is > 4 seconds between both timestamps)
 */
unsigned long ts_diff_ns(struct timespec *from_ts, struct timespec *to_ts) {
  return (to_ts->tv_sec - from_ts->tv_sec) * 1000000000 
       + (to_ts->tv_nsec - from_ts->tv_nsec);
}

/**
 * @brief A busy loop that spends some time stimulating the CPU's ALU
 * @param loops perform approximately 10*loops integer operations
 */
void busy_loop(unsigned long loops) {
    uint32_t x = 0;
    volatile uint32_t __attribute__((unused)) y = 0;
    for (unsigned long i = 0; i < loops; i++) {
      /* Knuth and H. W. Lewis' simple pseudo-random number generator */
      REPEAT_X10(x = 1664525ul * x + 1013904223ul); 
    }
    /* store the result in a volatile variable to prevent the compiler fom 
    optimizing away computation of a useless result */
    y = x;  
}

/** @brief How many loops (of the busy_loop function) for 1 nanosecond of computations */
double loops_per_ns = LOOPS_PER_NS;

/**
 * @brief Keep the CPU busy for specified duration
 * @param ns duration, in nanoseconds
 * @note Make sure calibrate() function is called before!
 */
void busy_loop_ns(unsigned long int ns) {
  unsigned long loops = (unsigned long)(ns * loops_per_ns);
  busy_loop(loops);
}

/**
 * @brief Keep the CPU busy for specified duration
 * @param ms duration, in milliseconds
 * @note Make sure calibrate() function is called before!
 */
void busy_loop_ms(unsigned long int ms) {
  busy_loop_ns(ms * 1000000);
}



/**
 * @brief Calibrate the loops_per_ns factor
 */
void calibrate() {
  if (loops_per_ns == 0.0f) { // not yet calibrated
    uint32_t loops;
    struct timespec start_ts;
    struct timespec fin_ts;
    unsigned long delta_ns;

    // First, identify an iteration count `loops` that represents ~ 1 to 2 milliseconds
    for (loops = 1; loops < UINT32_MAX / 2; loops *= 2) {
      clock_gettime(CLOCK_MONOTONIC, &start_ts);
      busy_loop(loops);
      clock_gettime(CLOCK_MONOTONIC, &fin_ts);
      delta_ns = ts_diff_ns(&start_ts, &fin_ts); 
    
      if (delta_ns > 1*1000*1000) {
        break;
      }
    }

    /* Now run again several times, take the shortest measure (presumably with 
    less scheduling interference) as a basis for calibration */
    for (int i = 0; i < 100; i++) {
      clock_gettime(CLOCK_MONOTONIC, &start_ts);
      busy_loop(loops);
      clock_gettime(CLOCK_MONOTONIC, &fin_ts);
      delta_ns = MIN(delta_ns, ts_diff_ns(&start_ts, &fin_ts)); 
    }
    loops_per_ns = loops * 1.0f / delta_ns;
    printf("Calibration: %.6f loops/ns", loops_per_ns);

  }
  /* Perform some statistics to evaluate the accuracy of our calibration */
  printf("Testing accuracy of a 5ms busy-loop");
  for (int i = 0; i < 20; i++) {
      busy_loop_ms(5);
  }
  printf("\n");  
}
