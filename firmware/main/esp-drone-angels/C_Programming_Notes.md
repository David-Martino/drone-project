# C Programming Notes

## Macros in the ESP Drone Codebase

This is an example of a macro being used: 

```
STATIC_MEM_TASK_ALLOC(systemTask, SYSTEM_TASK_STACKSIZE);
```

This macro happens to be defined at the file level (i.e., at the top of `system.c` instead of inside a function) and this macro gets substituted with these lines of code before compiling:

```
/**
 * @brief Allocate variables and stack for a task using static memory.
 *
 * Note: the stack memory is allocated in normal RAM while the buffers
 * are allocated in CCM RAM. The special properties of CCM RAM should not
 * have any impact on the behaviour.

 * @param NAME A name used as base name for the variables that are created
 * @param STACK_DEPTH The stack depth in nr of StackType_t entries.
 */
#define STATIC_MEM_TASK_ALLOC(NAME, STACK_DEPTH) \
  static const int osSys_ ## NAME ## StackDepth = (STACK_DEPTH); \
  static StackType_t osSys_ ## NAME ## StackBuffer[(STACK_DEPTH)]; \
  NO_DMA_CCM_SAFE_ZERO_INIT static StaticTask_t osSys_ ## NAME ## TaskBuffer;
```

Hence, it essentially just declares static global variables for the stack of the systemTask task. 

Here is another example of a macro being used:

```
STATIC_MEM_TASK_CREATE(systemTask, systemTask, SYSTEM_TASK_NAME, NULL, SYSTEM_TASK_PRI)
```

Like all macros, it also gets substituted with lines of code before compiling:

```
/**
 * @brief Create a task using static memory
 *
 * The task is created under the assumption that STATIC_MEM_TASK_ALLOC() has been
 * used to define the required variables and buffers.
 *
 * @param NAME A name used as base name for the variables, same name that was used in STATIC_MEM_TASK_ALLOC()
 * @param FUNCTION The function that implements the task
 * @param TASK_NAME A descriptive name for the task
 * @param PARAMETERS Passed on as argument to the function implementing the task
 * @param PRIORITY The task priority
 */
#define STATIC_MEM_TASK_CREATE(NAME, FUNCTION, TASK_NAME, PARAMETERS, PRIORITY) xTaskCreateStatic((FUNCTION), (TASK_NAME), osSys_ ## NAME ## StackDepth, (PARAMETERS), (PRIORITY), osSys_ ## NAME ## StackBuffer, &osSys_ ## NAME ## TaskBuffer)
```

However, this macro was used within a function called `systemLaunch()` and it gets replaced with a function called `xTaskCreateStatic()`.

This means that `xTaskCreateStatic()` will create the `systemTask` task at run time when `systemLaunch()` is called, and the stack for the task will already be reserved in memory due to the first macro. 
