/**
 *
 * @file Defines the public interface for controlling "simulated interrupts".
 */
#ifndef INTERRUPTS_H
#define INTERRUPTS_H
#include <signal.h>
#include <stdio.h>

/**
 * How frequently this process will be interrupted.
 */
#define INTERRUPTS_SIGNAL_INTERVAL 200

/**
 * Enum specifying the state of interrupts.
 */
typedef enum
{
  INTERRUPTS_DISABLED = 0,
  INTERRUPTS_ENABLED = 1,
}  InterruptsState;

/**
 * Enum specifying the verbosity of outputs (logging) produced by interrupts.
 */
typedef enum
{
  INTERRUPTS_QUIET = 0,
  INTERRUPTS_VERBOSE = 1,
}  InterruptsOutput;

/**
 * Initialize the library.
 *
 * This must be called before using other functions in this header.
 *
 * @return 0 on success, -1 otherwise.
 */
void
 InterruptsInit(void);

/**
 * Set whether interrupts should be enabled or disabled.
 *
 * @return The state of interrupts before the call to this function.
 */
 InterruptsState
 InterruptsSet( InterruptsState state);

/**
 * Enable interrupts.
 *
 * @return The state of interrupts before the call to this function.
 */
 InterruptsState
 InterruptsEnable(void);

/**
 * Disable interrupts.
 *
 * @return The state of interrupts before the call to this function.
 */
 InterruptsState
 InterruptsDisable(void);

/**
 * @return whether interrupts are enabled (1) or not (0).
 */
int
 InterruptsAreEnabled(void);

/**
 * Set the verbosity of logging.
 */
void
 InterruptsSetLogLevel( InterruptsOutput level);

/**
 * Print to stdout safely (i.e., without being interrupted).
 *
 * This function can be called as if it were printf.
 */
int
 InterruptsPrintf(const char* fmt, ...);

#endif // INTERRUPTS_H
