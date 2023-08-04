/**
 * @file An application where all threads sleep except for one, non-main thread.
 */
#include <stdlib.h>

#include "interrupts.h"
#include "thread.h"

// Number of threads to create
#define THREAD_COUNT 32
// Thread IDs
int thread_storage[THREAD_COUNT];
// Barrier
int ALL_THREADS_CREATED = 0;

void
f_spin_and_join(int num)
{
  // Generate a random amount of time to spin for
  double const rand = ((double)random()) / RAND_MAX * 1000000;

  // Wait until the main thread has created all other threads before continuing
  while (__sync_fetch_and_add(&ALL_THREADS_CREATED, 0) < 1)
    ;

  // Spin, using up the CPU
  ThreadSpin((int)rand);

  if (num == 0) {
    // wait until all other threads, including the main one, are asleep
    while (ThreadYield() != ThreadId())
      ;
  } else {
    // sleep until the previous thread has finished
    int exit_code;
    Tid const tid = ThreadJoin(thread_storage[num - 1], &exit_code);

    ThreadSpin((int)rand / 10);
    InterruptsPrintf(
      "TID(%d) waited for TID(%d), which exited with %d\n",
      ThreadId(),
      tid,
      exit_code);
  }

  ThreadExit(num + THREAD_COUNT);
}

void
run_spin_join(void)
{
  srandom(123);

  for (long i = 0; i < THREAD_COUNT; i++) {
    thread_storage[i] =
      ThreadCreate((void (*)(void*))f_spin_and_join, (void*)i);
  }

  __sync_fetch_and_add(&ALL_THREADS_CREATED, 1);

  int exit_code;
  Tid tid =  ThreadJoin(thread_storage[THREAD_COUNT - 1], &exit_code);
  InterruptsPrintf("TID(%d) waited for TID(%d), which exited with %d\n",
                          ThreadId(),
                          tid,
                          exit_code);
}

int
main(void)
{
  // Initialize the user-level thread package
  ThreadInit();
  // Initialize and enable interrupts
  InterruptsInit();
  // Uninterrupted prints are expensive, keep the interrupts logging quiet
  InterruptsSetLogLevel( INTERRUPTS_QUIET);

  run_spin_join();

  return 0;
}