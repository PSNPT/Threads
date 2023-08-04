/**
 * @file An application that relies on preemption to pass a hot potato around.
 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include "interrupts.h"
#include "thread.h"

// Number of threads to create
#define THREAD_COUNT 64
// Approximately how long the main thread will spin
#define RUNTIME_DURATION 10000000

// Each thread tracks whether it has the hot potato
int thread_storage[THREAD_COUNT];
// One lock for the thread_storage array
int lock = 0;
// The original start time
struct timeval start_time;

void
report_pass(int from, int to)
{
  struct timeval pend, pdiff;
  gettimeofday(&pend, NULL);
  timersub(&pend, &start_time, &pdiff);

   InterruptsPrintf("%9.6f: hot potato passed from %d to %d.\n",
                          (float)pdiff.tv_sec + (float)pdiff.tv_usec / 1000000,
                          from,
                          to);
}

int
pass_hot_potato_randomly(int this_index)
{
  assert( InterruptsAreEnabled());

  double const rand = ((double)random()) / RAND_MAX;
  int const next_index = (int)(rand * THREAD_COUNT - 1);

  int err = __sync_bool_compare_and_swap(&lock, 0, 1);
  if (err == 0) {
    return 0; // Could not acquire the lock, try again later
  }

  if (thread_storage[this_index] == 1) {
    // We have the hot potato, pass it!
    thread_storage[this_index] = 0;
    thread_storage[next_index] = 1;

    report_pass(this_index, next_index);
  }

  err = __sync_bool_compare_and_swap(&lock, 1, 0);
  assert(err); // We should definitely be able to release the lock

  return 1;
}

int
pass_hot_potato(int this_index)
{
  assert( InterruptsAreEnabled());

  int const next_index = (this_index + 1) % THREAD_COUNT;

  int err = __sync_bool_compare_and_swap(&lock, 0, 1);
  if (!err) {
    return 0; // Could not acquire the lock, try again later
  }

  if (thread_storage[this_index] == 1) {
    thread_storage[this_index] = 0;
    thread_storage[next_index] = 1;
    report_pass(this_index, next_index);
  }

  err = __sync_bool_compare_and_swap(&lock, 1, 0);
  assert(err); // We should definitely be able to release the lock

  return 1;
}

void
f_potato(int this_index)
{
  assert( InterruptsAreEnabled());
  int pass = 1;

  while (1) {
    int const was_passed = pass_hot_potato(this_index);
    // int const was_passed = pass_hot_potato_randomly(this_index);
    pass += was_passed;

    ThreadSpin( INTERRUPTS_SIGNAL_INTERVAL * 2);
  }

  // This thread function must be killed to exit
}

void
run_hot_potato()
{
  srandom(123);

  InterruptsPrintf("Starting hot potato.\n");
  gettimeofday(&start_time, NULL);

  thread_storage[0] = 1; // Give the hot potato to index 0
  for (int i = 1; i < THREAD_COUNT; i++) {
    thread_storage[i] = 0; // All other indexes do not have the potato
  }

  Tid potato_tids[THREAD_COUNT];
  for (long i = 0; i < THREAD_COUNT; i++) {
    potato_tids[i] =  ThreadCreate((void (*)(void*))f_potato, (void*)i);
  }

  ThreadSpin(RUNTIME_DURATION);

  InterruptsPrintf("Killing all created threads.\n");
  for (int i = 0; i < THREAD_COUNT; i++) {
    assert( InterruptsAreEnabled());
    ThreadKill(potato_tids[i]);
  }

  InterruptsPrintf("Hot potato is done.\n");
}

int
main()
{
  // Initialize the user-level thread package
  ThreadInit();
  // Initialize and enable interrupts
  InterruptsInit();
  // Uninterrupted prints are expensive, keep the interrupts logging quiet
  InterruptsSetLogLevel( INTERRUPTS_QUIET);

  run_hot_potato();

  return 0;
}