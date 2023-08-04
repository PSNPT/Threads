#include "thread.h"

#include <ucontext.h>

#include <assert.h>
#include <sys/time.h>

#include <stdlib.h>  
  
#include <stdio.h>  
  
#include <string.h>  

#include "interrupts.h"

//****************************************************************************
// Private Definitions
//****************************************************************************

typedef enum
{
  INITIAL = 0,
  READY = 1,
  RUNNING = 2,
  BLOCKED = 3,
  ZOMBIE = 4,
  SLEEP = 5,
}  ThreadState;

/**
 * The Thread Control Block.
 */
typedef struct
{
  Tid identifier; 

  ucontext_t *context; 

  int state; 

  int* stack;

  int latest_join_exit_code; // Stores the exit code of the thread joined to

  int exit_code; // Own exit code

   WaitQueue* join_threads; // Queue of threads waiting for this threads exit call

} TCB;

typedef struct Node
{
  TCB *thread;

  struct Node *next;

} Node;

typedef struct
{
  Node *head;

} Queue;

/**
 * A wait queue.
 */
typedef struct  wait_queue_t
{
  Node *head;
}  WaitQueue;

//**************************************************************************************************
// Private Global Variables (Library State)
//**************************************************************************************************

int TID[ MAX_THREADS];

Queue *ready;
Queue *zombie;

TCB *running;

//**************************************************************************************************
// Helper Functions
//**************************************************************************************************

void
DisplayQueue(Queue *queue) // Debugging
{
  if(queue == NULL)
  {
    printf("Queue not initialized\n");
    fflush(stdout);
    return;
  }

  Node *curr = queue->head;
  int iter = 0;
  while(curr != NULL)
  {
    printf("Item %d: TCB identifier %d\n", iter, curr->thread->identifier);
    fflush(stdout);
    iter++;
    curr = curr->next;
  }
  return;
}


int
NumActive() // Number of active threads
{ 
  int sum = 0;

  for(int i = 0; i <  MAX_THREADS; i++){
    
    if(TID[i] == 1)
    {
      sum++;
    }

  }

  return sum;

}

int
FindVacant() // Find and return the first unused valid Tid
{
  for(int i = 0; i <  MAX_THREADS; i++){
    
    if(TID[i] == 0)
    {
      return i;
    }

  }

  return -1;
}

void
MyThreadStub(void (*f)(void *), void *arg)
{ 
   InterruptsEnable(); // When thread created, context has interrupts disabled -> Need to enable on start

  f(arg);
   ThreadExit(running->exit_code);

}

void
Enqueue(Queue *queue, TCB *thread) // Insert thread into the back of this queue
{
  Node *new = malloc(sizeof(Node));

  new->thread = thread;

  new->next = NULL;

 if (queue->head == NULL){
    queue->head = new;
 }

 else {

  Node *curr = queue->head;

  while(curr->next != NULL)
  {
    curr = curr->next;
  }

  curr->next = new;

 } 

}

void
EnqueueWait( WaitQueue *queue, TCB *thread) // WaitQueue type Enqueue
{
  Node *new = malloc(sizeof(Node));

  new->thread = thread;

  new->next = NULL;

 if (queue->head == NULL){
    queue->head = new;
 }

 else {

  Node *curr = queue->head;

  while(curr->next != NULL)
  {
    curr = curr->next;
  }

  curr->next = new;

 } 

}

TCB*
Dequeue(Queue *queue) // Remove first node of queue
{

  if(queue->head == NULL)
  {
    return NULL;
  }
  
  else 
  {
    Node *garbage = queue->head;

    TCB *ret = queue->head->thread;
    queue->head = queue->head->next;
    
    free(garbage);
    return ret;
  }

}

TCB*
DequeueWait( WaitQueue *queue) // WaitQueue type dequeue
{

  if(queue->head == NULL)
  {
    return NULL;
  }
  
  else 
  {
    Node *garbage = queue->head;

    TCB *ret = queue->head->thread;
    queue->head = queue->head->next;
    
    free(garbage);
    return ret;
  }

}

TCB*
ExtractWait( WaitQueue *queue, Tid tid) // Search queue (Including node's own WaitQueue's) for thread with identifier tid and remove it 
{
    Node *curr = queue->head;
    Node *left = NULL;
    Node *right = NULL;

    if(curr!= NULL)
    {
      right = curr->next;
    }

    while(curr != NULL && curr->thread->identifier != tid)
    { 

      TCB* temp = ExtractWait(curr->thread->join_threads, tid);
      if(temp != NULL)
      {
        return temp;
      }


      left = curr;
      curr = right;

      right = NULL;
      if(curr!= NULL)
      {
        right = curr->next;
      }
      
    }

    Node *target = curr;
    TCB *ret = NULL;

    if(target == NULL)
    {
      return NULL; // Empty queue or no thread with identifier as tid
    }

    else if(left == NULL)
    {
      queue->head = right; // target is the first node
    }

    else if(right == NULL)
    {
      left->next = NULL; // target is the last node
    }

    else
    {
      left->next = right; // target is surrounded by 2 nodes (middle)
    }

    ret = target->thread;
    free(target); // Free memory associated with the node containing extracted thread

    return ret;
}

TCB*
Extract(Queue *queue, Tid tid) // Non-WaitQueue version
{
    Node *curr = queue->head;
    Node *left = NULL;
    Node *right = NULL;

    if(curr!= NULL)
    {
      right = curr->next;
    }

    while(curr != NULL && curr->thread->identifier != tid)
    { 

      TCB* temp = ExtractWait(curr->thread->join_threads, tid);
      if(temp != NULL)
      {
        return temp;
      }


      left = curr;
      curr = right;

      right = NULL;
      if(curr!= NULL)
      {
        right = curr->next;
      }
      
    }

    Node *target = curr;
    TCB *ret = NULL;

    if(target == NULL)
    {
      return NULL; // Empty queue or no thread with identifier as tid
    }

    else if(left == NULL)
    {
      queue->head = right; // target is the first node
    }

    else if(right == NULL)
    {
      left->next = NULL; // target is the last node
    }

    else
    {
      left->next = right; // target is surrounded by 2 nodes (middle)
    }

    ret = target->thread;
    free(target); // Free memory associated with the node containing extracted thread

    return ret;
}

TCB*
Extract1D(Queue *queue, Tid tid) // Extract, but do not go into node's WaitQueue
{
    Node *curr = queue->head;
    Node *left = NULL;
    Node *right = NULL;

    if(curr!= NULL)
    {
      right = curr->next;
    }

    while(curr != NULL && curr->thread->identifier != tid)
    { 
      left = curr;
      curr = right;

      right = NULL;
      if(curr!= NULL)
      {
        right = curr->next;
      }
      
    }

    Node *target = curr;
    TCB *ret = NULL;

    if(target == NULL)
    {
      return NULL; // Empty queue or no thread with identifier as tid
    }

    else if(left == NULL)
    {
      queue->head = right; // target is the first node
    }

    else if(right == NULL)
    {
      left->next = NULL; // target is the last node
    }

    else
    {
      left->next = right; // target is surrounded by 2 nodes (middle)
    }

    ret = target->thread;
    free(target); // Free memory associated with the node containing extracted thread

    return ret;
}



TCB*
FindWait( WaitQueue *queue, Tid tid) // WaitQueue version
{
    Node *curr = queue->head;

    

    while(curr != NULL && curr->thread->identifier != tid)
    { 
      TCB* temp = FindWait(curr->thread->join_threads, tid);

      if(temp == NULL)
      {
        curr = curr->next;
      }
      else
      {
        return temp;
      }
    }

    if(curr == NULL)
    {
      return NULL;
    }

    return curr->thread;
}

TCB*
Find(Queue *queue, Tid tid) // Find thread with identifier tid in queue (and in node's WaitQueues)
{
    Node *curr = queue->head;


    while(curr != NULL && curr->thread->identifier != tid)
    { 
      TCB* temp = FindWait(curr->thread->join_threads, tid);

      if(temp == NULL)
      {
        curr = curr->next;
      }
      else
      {
        return temp;
      }
    }

    if(curr == NULL)
    {
      return NULL;
    }

    return curr->thread;
}



void
FreeThread(TCB *thread) // Free thread's stack, context, WaitQueue and self allocation in memory
{
  free(thread->stack); // Free thread's stack
  free(thread->context); // Free thread's context
   WaitQueueDestroy(thread->join_threads); // Free threads wait queue
  free(thread); // Free thread
}

void
FreeZombie() // Free memory associated with any threads with state ZOMBIE
{
  Node *curr = zombie->head;
  while(curr != NULL)
  {
    // THREAD
    FreeThread(curr->thread);

    // NODE
    Node *temp = curr;
    curr = curr->next;
    free(temp);
  }

  zombie->head = NULL;
}

void
GarbageCollection() // Free memory associated with programs= global variables
{ 
  // QUEUES
  FreeZombie();
  free(ready);
  free(zombie);
   WaitQueueDestroy(running->join_threads);

  free(running->context);

  free(running);
  
  // Cannot manually deallocate final stack
}


//**************************************************************************************************
// thread.h Functions
//**************************************************************************************************
int
 ThreadInit(void)
{ 
  memset(TID, 0, sizeof(TID)); // No Tid's being used on initialization

  ready = malloc(sizeof(Queue)); // Thread ready queue allocation
  if (ready == NULL)
  { 
    return  ERROR_OTHER;
  }
  ready->head = NULL;

  zombie = malloc(sizeof(Queue)); // Thread zombie queue allocation
  if (zombie == NULL)
  { 
    return  ERROR_OTHER;
  }
  zombie->head = NULL;

  running = malloc(sizeof(TCB)); // OS supplied TCB memory allocation
  if (running == NULL)
  {
    return  ERROR_OTHER;
  }



  TID[0] = 1; // OS supplied Tid = 0
  running->identifier = 0; 

  running->exit_code =  EXIT_CODE_NORMAL; // Default exit value if no errors raised

  running->state = RUNNING; // OS supplied thread currently running
  running->stack = NULL; // OS supplied thread stack already provided
  running->context = malloc(sizeof(ucontext_t)); // OS supplied thread context memory allocation
  if (running->context == NULL)
  {
    return  ERROR_OTHER;
  }
  running->join_threads =  WaitQueueCreate(); // OS supplied thread wait queue memory allocation
  if (running->join_threads == NULL)
  {
    return  ERROR_OTHER;
  }


  int err = atexit(GarbageCollection); // Set GarbageCollection as exit function
  if(err != 0)
  {
    return  ERROR_OTHER;
  }

  return 0;
}




Tid
 ThreadId(void)
{
  return running->identifier; // Return current running thread's Tid
}




Tid
 ThreadCreate(void (*f)(void*), void* arg)
{

  int const prev_state =  InterruptsDisable(); // Critical section begin

  FreeZombie(); // Deallocate all resources associated with Zombie threads

  if (NumActive() ==  MAX_THREADS) // Error:  MAX_THREADS concurrently exist
  {
     InterruptsSet(prev_state); // Set interrupt state to caller function's state
    return  ERROR_SYS_THREAD;
  }


  TCB *thread = malloc(sizeof(TCB)); // Memory allocation for new thread's TCB
  if (thread == NULL)
  {
     InterruptsSet(prev_state); // Set interrupt state to caller function's state
    return  ERROR_SYS_MEM;
  }



  thread->context = malloc(sizeof(ucontext_t)); // Memory allocation for new thread's context
  if (thread->context == NULL)
  {
    free(thread);
     InterruptsSet(prev_state); // Set interrupt state to caller function's state
    return  ERROR_SYS_MEM;
  }


  thread->stack = malloc( THREAD_STACK_SIZE); // Memory allocation for new thread's stack
  if (thread->stack == NULL)
  { 
    free(thread->context);
    free(thread);
     InterruptsSet(prev_state); // Set interrupt state to caller function's state
    return  ERROR_SYS_MEM;
  }

  thread->join_threads =  WaitQueueCreate(); // Memory allocation for new thread's wait queue
  if (thread->join_threads == NULL)
  { 
    free(thread->context);
    free(thread->stack);
    free(thread);
     InterruptsSet(prev_state); // Set interrupt state to caller function's state
    return  ERROR_SYS_MEM;
  }


  
  unsigned long top = (unsigned long)thread->stack +  THREAD_STACK_SIZE - 8;
  // 16 byte alignment: -8 from push misaligns by 8 bytes -> subtract another 8
  
  
  
  if (getcontext(thread->context) == -1) // Context stored in new thread's context
  { 
    FreeThread(thread);
     InterruptsSet(prev_state); // Set interrupt state to caller function's state
    return  ERROR_OTHER;
  }


  thread->identifier = FindVacant(); // Finds earliest vacant Tid & Sets new thread's identifier to it
  TID[thread->identifier] = 1; // Signal the identifier is taken 

  thread->exit_code =  EXIT_CODE_NORMAL; // Default exit value if no errors

  thread->context->uc_mcontext.gregs[REG_RIP] = (unsigned long) MyThreadStub; // Begin execution at Stub
  thread->context->uc_mcontext.gregs[REG_RSP] = (unsigned long) top; // Stack pointer set to top of allocated region
  thread->context->uc_mcontext.gregs[REG_RDI] = (unsigned long) f; // First argument to Stub
  thread->context->uc_mcontext.gregs[REG_RSI] = (unsigned long) arg; // Second argument to Stub
  
  thread->state = READY; // Signal the thread is ready for processing

  // Thread creation finished

  Enqueue(ready, thread); // Insert thread into queue

   InterruptsSet(prev_state); // Set interrupt state to caller function's state

  return thread->identifier; // Return threads identifier
}




void
 ThreadExit(int exit_code)
{ 
  int const prev_state =  InterruptsDisable(); // Critical section begin

  if(ready->head == NULL && running->join_threads->head == NULL) // Last active thread
  { 
    running->exit_code = exit_code;
     InterruptsSet(prev_state); // Set interrupt state to caller function's state
    exit(exit_code);
  }

  running->state = ZOMBIE; // Signal the thread is finished
  running->exit_code = exit_code; // Signal the thread voluntarily exited with given exitcode
  TID[running->identifier] = 0; // Signal the Tid is free

  // Thread exiting, wake up any dependent threads

   ThreadWakeAll(running->join_threads);

  // Thread exiting, wake up any dependent threads

  if ( ThreadYield() < 0) // Switch to next ready thread
  {
     InterruptsSet(prev_state); // Set interrupt state to caller function's state
    exit( EXIT_CODE_FATAL); // Switch unsuccessful
  }
}




Tid
 ThreadKill(Tid tid)
{ 
  int const prev_state =  InterruptsDisable(); // Critical section begin

  if(tid == running->identifier) // Cannot kill self
  { 
     InterruptsSet(prev_state); // Set interrupt state to caller function's state
    return  ERROR_THREAD_BAD;
  }

  if(tid < 0 || tid >=  MAX_THREADS) // Check for invalidity of tid
  { 
     InterruptsSet(prev_state); // Set interrupt state to caller function's state
    return  ERROR_TID_INVALID;

  }

  if(TID[tid] == 0) // Check if tid is not active
  {
     InterruptsSet(prev_state); // Set interrupt state to caller function's state
    return  ERROR_SYS_THREAD;
  }

  TCB* target;

  if(ready->head == NULL)
  {
    target = ExtractWait(running->join_threads, tid); // Searching for tid in own wait queue

  }
  else
  {
    TCB* target1;
    TCB* target2;
    target1 = ExtractWait(running->join_threads, tid); // Searching for tid in own wait queue
    target2 = Extract(ready, tid); // Searching for tid in ready queue

    if(target1 == NULL)
    {
      target = target2;
    }
    else
    {
      target = target1;
    }
  }

  target->state = ZOMBIE; // Signal the thread is dead
  target->exit_code =  EXIT_CODE_KILL; // Signal the thread exited via a kill signal
  TID[target->identifier] = 0; // Signal the Tid is free

  // Thread exiting, wake up any dependent threads

   ThreadWakeAll(target->join_threads);

  // Thread exiting, wake up any dependent threads

  Enqueue(zombie, target); // Queue thread to kill up for garbage collection

   InterruptsSet(prev_state); // Set interrupt state to caller function's state
  return tid;
}



Tid
 ThreadYield()
{ 
  int const prev_state =  InterruptsDisable(); // Critical section begin

  if(ready->head == NULL) // If no other threads to yield to, yield to self
  {
     InterruptsSet(prev_state); // Set interrupt state to caller function's state
    return running->identifier;
  }

  Tid peek = ready->head->thread->identifier; // Find identifier of the thread at the front of the queue

   ThreadYieldTo(peek); // Yield to said thread
  
   InterruptsSet(prev_state); // Set interrupt state to caller function's state

  return peek;
}




Tid
 ThreadYieldTo(Tid tid)
{ 
  int const prev_state =  InterruptsDisable(); // Critical section begin

  if(tid == running->identifier) // Yield to self
  {
     InterruptsSet(prev_state); // Set interrupt state to caller function's state
    return tid;
  }

  if(tid < 0 || tid >=  MAX_THREADS) // Check for invalidity of tid
  {
     InterruptsSet(prev_state); // Set interrupt state to caller function's state
    return  ERROR_TID_INVALID;
  }

  if(TID[tid] == 0) // Check if tid is not active
  {
     InterruptsSet(prev_state); // Set interrupt state to caller function's state
    return  ERROR_THREAD_BAD;
  }
  

  volatile int yielded = 0; // Flag for determining if running thread should perform a yield

  int get = getcontext(running->context); // Save current context into currently running thread's context
  if(get == -1)
  {
     InterruptsSet(prev_state); // Set interrupt state to caller function's state
    return  ERROR_OTHER;
  }

  FreeZombie(); // Ensure garbage collection of fully exited threads

  if(yielded == 0) // Perform yield
  {
    // Begin search and removal of specified thread with identifier tid

    TCB *target = Extract1D(ready, tid);

    // End search and removal of specified thread with thread with identifier tid

    if(target == NULL) // Target not found in ready queue, but Tid valid, and not exit/kill => MUST be sleeping
    {
       InterruptsSet(prev_state); // Set interrupt state to caller function's state
      return  ERROR_THREAD_BAD;
    }

    // Target found
    
    if (running->state == ZOMBIE) // If thread is yielding due to an exit call
    {
      Enqueue(zombie, running);
    }
    else if(running->state == RUNNING) // If thread is yielding and will continue execution in the future
    {
      running->state = READY;
      Enqueue(ready, running);
    }


    yielded = 1; // Signal that currently running thread should not perform yield on return

    TCB *temp = running; // In case setcontext fails, to revert to this TCB

    

    running = target; // Sets currently running thread to thread with identifier as tid
    running->state = RUNNING; // Sets said thread's state to RUNNING

    int set = setcontext(running->context); // Perform the actual context swap
    
    if(set == -1)
    {
       InterruptsSet(prev_state); // Set interrupt state to caller function's state
      running = temp;
      return  ERROR_OTHER;
    }

  }

   InterruptsSet(prev_state); // Set interrupt state to caller function's state
  return tid; 
}


 WaitQueue*
 WaitQueueCreate(void)
{
  int const prev_state =  InterruptsDisable(); // Critical section begin

   WaitQueue *new = malloc(sizeof( WaitQueue)); // Allocate memory for wait queue
  
  if(new != NULL)
  {
    new->head = NULL;
  }

   InterruptsSet(prev_state); // Set interrupt state to caller function's state

  return new; // Null if malloc failed
}

int
 WaitQueueDestroy( WaitQueue* queue)
{
  int const prev_state =  InterruptsDisable(); // Critical section begin

  if(queue->head != NULL) // If there exist item(s) in wait queue, error
  {
    return  ERROR_OTHER; 
  }

  free(queue); // No items in queue, free the queue

   InterruptsSet(prev_state); // Set interrupt state to caller function's state

  return 0;
}

void
 ThreadSpin(int duration)
{
  struct timeval start, end, diff;

  int ret = gettimeofday(&start, NULL);
  assert(!ret);

  while (1) {
    ret = gettimeofday(&end, NULL);
    assert(!ret);
    timersub(&end, &start, &diff);

    if ((diff.tv_sec * 1000000 + diff.tv_usec) >= duration) {
      return;
    }
  }
}

int
 ThreadSleep( WaitQueue* queue)
{ 
  int const prev_state =  InterruptsDisable(); // Critical section begin
  
  assert(queue != NULL); // (pre) assertion
  
  if(ready->head == NULL) // If no other threads can run, error
  {
     InterruptsSet(prev_state); // Set interrupt state to caller function's state
    return  ERROR_SYS_THREAD;
  }

  running->state = SLEEP; // Signal the thread is sleeping
  EnqueueWait(queue, running); // Queue up the current thread

  int ret =   ThreadYield(); // Context switch to some other thread

   InterruptsSet(prev_state); // Set interrupt state to caller function's state

  return ret; // Return the id of the thread yielded to
}

int
 ThreadWakeNext( WaitQueue* queue)
{
  int const prev_state =  InterruptsDisable(); // Critical section begin
  
  assert(queue != NULL); // (pre) assertion

  if(queue->head == NULL) // wait queue empty
  {
     InterruptsSet(prev_state); // Set interrupt state to caller function's state
    return 0;
  }

  TCB *woke = DequeueWait(queue); // Remove first node from wait queue;
  woke->state = READY; // Set the threads state to ready
  woke->latest_join_exit_code = running->exit_code; // Store latest parent exit code 
  Enqueue(ready, woke); // Add node to end of ready queue;

   InterruptsSet(prev_state); // Set interrupt state to caller function's state

  return 1; // Exactly one thread has been woken up and queued in ready
}

int
 ThreadWakeAll( WaitQueue* queue)
{
  int const prev_state =  InterruptsDisable(); // Critical section begin
  
  assert(queue != NULL); // (pre) assertion

  int numwoken = 0;

  while( ThreadWakeNext(queue) == 1) // Keep calling  ThreadWakeNext till all threads are dequeued
  {
    numwoken++;
  }

   InterruptsSet(prev_state); // Set interrupt state to caller function's state

  return numwoken;
}


int
 ThreadJoin(Tid tid, int* exit_code)
{ 

  int const prev_state =  InterruptsDisable(); // Critical section begin
  
    if(tid == running->identifier) // Cannot join self
  { 
     InterruptsSet(prev_state); // Set interrupt state to caller function's state
    return  ERROR_THREAD_BAD;
  }

  if(tid < 0 || tid >=  MAX_THREADS) // Check for invalidity of tid
  { 
     InterruptsSet(prev_state); // Set interrupt state to caller function's state
    return  ERROR_TID_INVALID;

  }

  if(TID[tid] == 0) // Check if tid is not active (Zombie/Uninit <=> TID[tid] = 0)
  {
     InterruptsSet(prev_state); // Set interrupt state to caller function's state
    return  ERROR_SYS_THREAD;
  }


  TCB *joinTo = Find(ready, tid); // Find pointer to thread with identifier tid

  if(joinTo == NULL)
  {
     InterruptsSet(prev_state); // Set interrupt state to caller function's state
    return  ERROR_SYS_THREAD;
  }

   ThreadSleep(joinTo->join_threads); // Put to sleep waiting on joinTo

  *(exit_code) = running->latest_join_exit_code; // Latest parents code

   InterruptsSet(prev_state); // Set interrupt state to caller function's state

  return tid;
}
