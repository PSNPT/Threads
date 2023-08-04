/**
 *
 * @file Defines the public interface for the library.
 */
#ifndef THREAD_H
#define THREAD_H

/**
 * Error codes for the library
 */
typedef enum
{
 ERROR_TID_INVALID = -1,
 ERROR_THREAD_BAD = -2,
 ERROR_SYS_THREAD = -3,
 ERROR_SYS_MEM = -4,
 ERROR_OTHER = -5
} ThreadError;

/**
 * Error codes for the library
 */
typedef enum
{
 EXIT_CODE_NORMAL = 0,
 EXIT_CODE_FATAL = -1,
 EXIT_CODE_KILL = -999,
} ExitCode;

/**
 * The maximum number of threads supported by the library.
 */
#define MAX_THREADS 256

/**
 * The minimum stack size, in bytes, of a thread.
 */
#define THREAD_STACK_SIZE 32768

/**
 * The identifier for a thread. Valid ids are non-negative and less than
 * MAX_THREADS.
 */
typedef int Tid;

/**
 * Initialize the user-level thread library.
 *
 * This must be called before using other functions in this library.
 *
 * @return 0 on success, ERROR_OTHER otherwise.
 */
int
ThreadInit(void);

/**
 * Get the identifier of the calling thread.
 *
 * @return The calling thread's identifier.
 */
Tid
ThreadId(void);

/**
 * Create a new thread that runs the function f with the argument arg.
 *
 * This function may fail if:
 * - no more threads can be created (ERROR_SYS_THREAD), or
 * - there is no more memory available (ERROR_SYS_MEM), or
 * - something unexpected failed (ERROR_OTHER)
 *
 * @param f A pointer to the function that this thread will execute.
 * @param arg The argument passed to f.
 *
 * @return If successful, the new thread's identifier. Otherwise, the
 * appropriate error code.
 */
Tid
ThreadCreate(void (*f)(void*), void* arg);

/**
 * Suspend the calling thread and run the next ready thread. The calling thread
 * will be scheduled again after all *currently* ready threads have run.
 *
 * When the calling thread is the only ready thread, it yields to itself.
 *
 * @return The thread identifier yielded to.
 */
Tid
ThreadYield(void);

/**
 * Suspend the calling thread and run the thread with identifier tid. The
 * calling thread will be scheduled again after all *currently* ready threads
 * have run.
 *
 * This function may fail if:
 * - the identifier is invalid (ERROR_TID_INVALID), or
 * - the identifier is valid but that thread cannot run
 * (ERROR_THREAD_BAD)
 *
 * @param tid The identifier of the thread to run.
 *
 * @return If successful, the identifier yielded to. Otherwise, the appropriate
 * error code.
 */
int
ThreadYieldTo(Tid tid);

/**
 * Exit the calling thread. If the caller is the last thread in the system, then
 * the program exits with the given exit code. If the caller is not the last
 * thread in the system, then the next ready thread will be run.
 *
 * This function may fail when switching to another ready thread. In this case,
 * the program exits with exit code -1.
 *
 * @param exit_code The code to exit the process with.
 *
 * @return This function does not return.
 */
void
ThreadExit(int exit_code);

/**
 * Kill the thread whose identifier is tid.
 *
 * The thread that is killed should "exit" with code -999.
 *
 * The thread being killed may be invalid. An invalid thread (not to be
 * confused with an invalid identifier) is a thread that is inactive
 * (e.g., the thread was never created or the thread was cleaned up already).
 *
 * Note: For this function, a "zombie" thread is not "invalid"
 *
 * This function may fail if:
 * - the identifier is invalid (ERROR_TID_INVALID), or
 * - the identifier is of the calling thread (ERROR_THREAD_BAD), or
 * - the thread is invalid (ERROR_SYS_THREAD)
 *
 * @param tid The identifier of the thread to kill.
 *
 * @return If successful, the killed thread's identifier. Otherwise, the
 * appropriate error code.
 */
int
ThreadKill(Tid tid);

/**
 * A (FIFO) queue of waiting threads.
 *
 * Representation Invariants:
 * - None of the threads in the wait queue are currently running
 * - A thread cannot be in more than one wait queue at a time
 */
typedef struct wait_queue_t WaitQueue;

/**
 * Create an empty queue for created threads.
 *
 * The queue created by this function must be freed using
 * WaitQueueDestroy.
 *
 * @return If successful, a pointer to newly allocated queue. Otherwise, NULL.
 */
WaitQueue*
WaitQueueCreate(void);

/**
 * Destroy the queue, freeing up allocated memory.
 *
 * This function may fail if:
 * - the queue is not empty (ERROR_OTHER)
 *
 * @return If successful, 0. Otherwise, the appropriate error code.
 *
 * @pre queue is not NULL
 */
int
WaitQueueDestroy(WaitQueue* queue);

/**
 * Spin for some amount of time.
 *
 * @param duration The time to spin in microseconds.
 */
void
ThreadSpin(int duration);

/**
 * Suspend the calling thread, enqueueing it on queue, and run the next ready
 * thread.
 *
 * This function may fail if:
 * - there are no other threads that can run (ERROR_SYS_THREAD)
 *
 * @param queue The wait queue that the calling thread should be added to.
 *
 * @return If successful, the identifier of the thread that ran. Otherwise, the
 * appropriate error code.
 *
 * @pre queue is not NULL
 */
int
ThreadSleep(WaitQueue* queue);

/**
 * Wake up the first thread in queue (and move it to the ready queue).
 *
 * The calling thread continues to execute (i.e., it is not suspended).
 *
 * @param queue The wait queue to dequeue.
 *
 * @return The number of threads woken up, which can be 0.
 *
 * @pre queue is not NULL
 */
int
ThreadWakeNext(WaitQueue* queue);

/**
 * Wake up all threads in queue in FIFO order (and move them to the ready
 * queue).
 *
 * The calling thread continues to execute (i.e., it is not suspended).
 *
 * @param queue The wait queue to dequeue.
 *
 * @return The number of threads woken up, which can be 0.
 *
 * @pre queue is not NULL
 */
int
ThreadWakeAll(WaitQueue* queue);

/**
 * Suspend the calling thread until the thread with identifier tid exits. If
 * the thread has already exited, this function returns immediately.
 *
 * The thread being waited for may be invalid. An invalid thread (not to be
 * confused with an invalid identifier) is a thread that is currently being
 * cleaned up (e.g., a zombie) or a thread that is inactive (e.g., the thread
 * was never created or the thread was cleaned up already).
 *
 * This function also copies the exit status of the target thread (tid) to
 * exit_code.
 *
 * This function may fail if:
 * - the identifier is invalid (ERROR_TID_INVALID), or
 * - the identifier is of the calling thread (ERROR_THREAD_BAD), or
 * - the thread is not valid (ERROR_SYS_THREAD)
 *
 * @param tid The identifier of the thread to wait for.
 * @param exit_code The code the thread that finished exited with.
 *
 * @return If successful, the identifier of the thread that exited. Otherwise,
 * the appropriate error code.
 *
 * @pre exit_code is not NULL
 */
int
ThreadJoin(Tid tid, int* exit_code);

#endif /* THREAD_H */
