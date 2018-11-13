/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package trclib;

import java.util.ArrayList;
import java.util.HashSet;

/**
 * This class provides methods for the callers to register/unregister cooperative multi-tasking tasks. It manages
 * these tasks and will work with the cooperative multi-tasking scheduler to run these tasks.
 */
public class TrcTaskMgr
{
    private static final String moduleName = "TrcTaskMgr";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private static TrcDbgTrace dbgTrace = null;

    private static final long INPUT_THREAD_INTERVAL = 20;       // in msec
    private static final long OUTPUT_THREAD_INTERVAL = 20;      // in msec
    private static final long taskNanoTimeThreshold = 10000000; // 10 msec

    /**
     * These are the task type TrcTaskMgr supports:
     */
    public enum TaskType
    {
        /**
         * START_TASK is called one time before a competition mode is about to start.
         */
        START_TASK(0),

        /**
         * STOP_TASK is called one time before a competition mode is about to end.
         */
        STOP_TASK(1),

        /**
         * PREPERIODIC_TASK is called periodically at a rate about 50Hz before runPeriodic() on the main robot thread.
         */
        PREPERIODIC_TASK(2),

        /**
         * POSTPERIODIC_TASK is called periodically at a rate about 50Hz after runPeriodic() on the main robot thread.
         */
        POSTPERIODIC_TASK(3),

        /**
         * PRECONTINUOUS_TASK is called periodically at a rate as fast as the scheduler is able to loop and is run
         * before runContinuous() on the main robot thread.
         */
        PRECONTINUOUS_TASK(4),

        /**
         * POSTCONTINUOUS_TASK is called periodically at a rate as fast as the schedule is able to loop and is run
         * after runContinuous() on the main robot thread.
         */
        POSTCONTINUOUS_TASK(5),

        /**
         * INPUT_TASK is called periodically at a rate about 50Hz on its own thread. Typically, it runs code that
         * reads sensor input.
         */
        INPUT_TASK(6),

        /**
         * OUTPUT_TASK is called periodically at a rate about 50Hz on its own thread. Typically, it runs code that
         * updates the state of actuators.
         */
        OUTPUT_TASK(7),

        /**
         * STANDALONE_TASK is called periodically at the specified interval on its own thread. Typically, code that
         * may block for a long time requires its own thread so that it doesn't degrade the performance of the other
         * threads.
         */
        STANDALONE_TASK(8);

        public int value;

        TaskType(int value)
        {
            this.value = value;
        }   //TaskType

    }   //enum TaskType

    /**
     * Any class that is registering a task must implement this interface.
     */
    public interface Task
    {
        /**
         * This method is called at the appropriate time this task is registered for.
         *
         * StartTask:
         *  This contains code that initializes the task before a competition mode is about to start and is run on
         *  the main robot thread. Typically, if the task is a robot subsystem, you may put last minute mode specific
         *  initialization code here. Most of the time, you don't need to register StartTask because all initialization
         *  is done in initRobot(). But sometimes, you may want to delay a certain initialization until right before
         *  competition starts. For example, you may want to reset the gyro heading right before competition starts to
         *  prevent drifting.
         *
         * StopTask:
         *  This contains code that cleans up the task before a competition mode is about to end and is run on the main
         *  robot thread. Typically, if the task is a robot subsystem, you may put code to stop the robot here. Most of
         *  the time, you don't need to register StopTask because the system will cut power to all the motors after a
         *  competition mode has ended.
         *
         * PrePeriodicTask:
         *  This contains code that runs before runPeriodic() is called on the main robot thread. Typically, you will
         *  put code that deals with any input or sensor readings here that runPeriodic() may depend on.
         *
         * PostPeriodicTask:
         *  This contains code that runs after runPeriodic() is called on the main robot thread. Typically, you will
         *  put code that deals with actions such as programming the motors here that may depend on the result produced
         *  by runPeriodic().
         *
         * PreContinuousTask:
         *  This contains code that runs before runContinuous() is called on the main robot thread. Typically, you will
         *  put code that deals with any input or sensor readings that requires more frequent processing here such as
         *  integrating the gyro rotation rate to heading.
         *
         * PostContinuousTask:
         *  This contains code that runs after runContinuous() is called on the main robot thread. Typically, you will
         *  put code that deals with actions that requires more frequent processing.
         *
         * InputTask:
         *  This contains code that runs periodically on the input thread. Typically, you will put code that deals with
         *  any input or sensor readings that may otherwise degrade the performance of the main robot thread.
         *
         * OutputTask:
         *  This contains code that runs periodically on the output thread. Typically, you will put code that deals
         *  with actions that may otherwise degrade the performance of the main robot thread.
         *
         * StandaloneTask:
         *  This contains code that will run on its own thread at the specified task interval. Typically, you will
         *  put code that may take a long time to execute and could affect the performance of the main robot thread.
         *
         * @param taskType specifies the type of task being run. This may be useful for handling multiple task types.
         * @param runMode specifies the competition mode that is about to end (e.g. Autonomous, TeleOp, Test).
         */
        void runTask(TaskType taskType, TrcRobot.RunMode runMode);

    }   //interface Task

    /**
     * This class implements TaskObject that will be created whenever a class is registered as a cooperative
     * multi-tasking task. The created task objects will be entered into an array list of task objects to be
     * scheduled by the scheduler.
     */
    public static class TaskObject
    {
        private final String taskName;
        private final Task task;
        private HashSet<TaskType> taskTypes;
        private long[] taskTotalNanoTimes = new long[TaskType.values().length];
        private int[] taskTimeSlotCounts = new int[TaskType.values().length];
        private TrcPeriodicThread<Object> taskThread = null;

        /**
         * Constructor: Creates an instance of the task object with the given name
         * and the given task type.
         *
         * @param taskName specifies the instance name of the task.
         * @param task specifies the object that implements the TrcTaskMgr.Task interface.
         */
        private TaskObject(final String taskName, Task task)
        {
            this.taskName = taskName;
            this.task = task;
            taskTypes = new HashSet<>();
            for (int i = 0; i < TaskType.values().length; i++)
            {
                taskTotalNanoTimes[i] = 0;
                taskTimeSlotCounts[i] = 0;
            }
        }   //TaskObject

        /**
         * This method returns the instance name of the task.
         *
         * @return instance name of the class.
         */
        public String toString()
        {
            return taskName;
        }   //toString

        /**
         * This method adds the given task type to the task object.
         *
         * @param type specifies the task type.
         * @param taskInterval specifies the periodic interval for STANDALONE_TASK, ignore for any other task types.
         *                     If zero interval is specified, the task will be run in a tight loop.
         * @return true if successful, false if the task with that task type is already registered in the task list.
         */
        public synchronized boolean registerTask(TaskType type, long taskInterval)
        {
            if (type == TaskType.STANDALONE_TASK && taskInterval < 0)
            {
                throw new IllegalArgumentException("taskInterval must be greater than or equal to 0.");
            }

            boolean added = taskTypes.add(type);

            if (added)
            {
                if (type == TaskType.STANDALONE_TASK)
                {
                    taskThread = new TrcPeriodicThread<>(taskName, this::standaloneTask, this);
                    taskThread.setProcessingInterval(taskInterval);
                    taskThread.setTaskEnabled(true);
                }
                else
                {
                    taskThread = null;
                    if (type == TaskType.INPUT_TASK)
                    {
                        //
                        // There is only one global input thread. All INPUT_TASKs run on this thread.
                        // The input thread is created on first registration, so create it if not already.
                        //
                        TrcTaskMgr.getInstance().startInputThread();
                    }
                    else if (type == TaskType.OUTPUT_TASK)
                    {
                        //
                        // There is only one global output thread. All OUTPUT_TASKs run on this thread.
                        // The output thread is created on first registration, so create it if not already.
                        //
                        TrcTaskMgr.getInstance().startOutputThread();
                    }
                }
            }

            return added;
        }   //registerTask

        /**
         * This method adds the given task type to the task object.
         *
         * @param type specifies the task type.
         * @return true if successful, false if the task with that task type is already registered in the task list.
         */
        public boolean registerTask(TaskType type)
        {
            return registerTask(type, 0);
        }   //registerTask

        /**
         * This method removes the given task type from the task object.
         *
         * @param type specifies the task type.
         * @return true if successful, false if the task with that type is not found the task list.
         */
        public synchronized boolean unregisterTask(TaskType type)
        {
            if (type == TaskType.STANDALONE_TASK && taskThread != null)
            {
                taskThread.terminateTask();
            }
            taskThread = null;

            return taskTypes.remove(type);
        }   //unregisterTask

        /**
         * This method checks if the given task type is registered with this task object.
         *
         * @param type specifies the task type to be checked against.
         * @return true if this task is registered as the given type, false otherwise.
         */
        private synchronized boolean hasType(TaskType type)
        {
            return taskTypes.contains(type);
        }   //hasType

        /**
         * This method returns the class object that was associated with this task object.
         *
         * @return class object associated with the task.
         */
        private Task getTask()
        {
            return task;
        }   //getTask

        /**
         * This method returns the task interval for TaskType.STANDALONE_TASK.
         *
         * @return task interval in msec. If there is no STANDALONE_TASK type in the task object, zero is returned.
         */
        public synchronized long getTaskInterval()
        {
            return taskThread != null? taskThread.getProcessingInterval(): 0;
        }   //getTaskInterval

        /**
         * This method sets the task interval for TaskType.STANDALONE_TASK. It has no effect for any other types.
         *
         * @param taskInterval specifies the periodic interval for STANDALONE_TASK, ignore for any other task types.
         *                     If zero interval is specified, the task will be run in a tight loop.
         */
        public synchronized void setTaskInterval(long taskInterval)
        {
            if (taskThread != null)
            {
                taskThread.setProcessingInterval(taskInterval);
            }
        }   //setTaskInterval

        /**
         * This method sets the task data for TaskType.STANDALONE_TASK. It has no effect for any other types.
         *
         * @param data specifies the thread data for STANDALONE_TASK, ignore for any other task types.
         */
        public synchronized void setTaskData(Object data)
        {
            if (taskThread != null)
            {
                taskThread.setData(data);
            }
        }   //setTaskData

        /**
         * This method runs the periodic standalone task.
         *
         * @param context specifies the context (task object).
         */
        private synchronized void standaloneTask(Object context)
        {
            final String funcName = "standaloneTask";
            TaskObject taskObj = (TaskObject)context;

            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "Executing StandaloneTask %s", taskObj);
            }

            long startNanoTime = TrcUtil.getCurrentTimeNanos();

            taskObj.getTask().runTask(TaskType.STANDALONE_TASK, TrcRobot.getRunMode());

            long elapsedTime = TrcUtil.getCurrentTimeNanos() - startNanoTime;
            taskObj.taskTotalNanoTimes[TaskType.STANDALONE_TASK.value] += elapsedTime;
            taskObj.taskTimeSlotCounts[TaskType.STANDALONE_TASK.value]++;

            if (debugEnabled)
            {
                if (elapsedTime > taskObj.getTaskInterval())
                {
                    dbgTrace.traceWarn(funcName, "%s.%s takes too long (%.3f)",
                            taskObj.taskName, TaskType.STANDALONE_TASK, elapsedTime/1000000000.0);
                }
            }
        }   //standaloneTask

    }   //class TaskObject

    private static TrcTaskMgr instance = null;
    private ArrayList<TaskObject> taskList = new ArrayList<>();
    private TrcPeriodicThread<Object> inputThread = null;
    private TrcPeriodicThread<Object> outputThread = null;

    /**
     * Constructor: Creates the global instance of task manager. There can only be one global instance of
     * task manager and is created on the first call to getInstance(). Any subsequent calls to getInstance()
     * will get the same global instance.
     */
    private TrcTaskMgr()
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }
    }   //TrcTaskMgr

    /**
     * This method returns the global instance of TrcTaskMgr. If Task Manager does not exist yet, one is created.
     *
     * @return global instance of TrcTaskMgr.
     */
    public static TrcTaskMgr getInstance()
    {
        if (instance == null)
        {
            instance = new TrcTaskMgr();
        }

        return instance;
    }   //getInstance

    /**
     * This method is called by registerTask for INPUT_TASK to create the input thread if not already.
     */
    private void startInputThread()
    {
        if (inputThread == null)
        {
            inputThread = startThread(moduleName + ".inputThread", this::inputTask, INPUT_THREAD_INTERVAL);
        }
    }   //startInputThread

    /**
     * This method is called by registerTask for OUTPUT_TASK to create the output thread if not already.
     */
    private void startOutputThread()
    {
        if (outputThread == null)
        {
            outputThread = startThread(moduleName + ".outputThread", this::outputTask, OUTPUT_THREAD_INTERVAL);
        }
    }   //startOutputThread

    /**
     * This method starts a periodic thread for processing tasks.
     *
     * @param instanceName specifies the instance name of the thread.
     * @param task specifies the task run by the thread.
     * @param interval specifies the processing interval of the task.
     * @return the created thread.
     */
    private TrcPeriodicThread<Object> startThread(
            String instanceName, TrcPeriodicThread.PeriodicTask task, long interval)
    {
        TrcPeriodicThread<Object> thread = new TrcPeriodicThread<>(instanceName, task, null);
        thread.setProcessingInterval(interval);
        thread.setTaskEnabled(true);

        return thread;
    }   //startThread

    /**
     * This method creates a TRC task. If the TRC task is registered as a STANDALONE task, it is run on a separately
     * created thread. Otherwise, it is run on the main robot thread as a cooperative multi-tasking task.
     *
     * @param taskName specifies the task name.
     * @param task specifies the Task interface for this task.
     * @return created task object.
     */
    public TaskObject createTask(final String taskName, Task task)
    {
        final String funcName = "createTask";
        TaskObject taskObj;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "taskName=%s", taskName);
        }

        taskObj = new TaskObject(taskName, task);
        taskList.add(taskObj);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", taskObj);
        }

        return taskObj;
    }   //createTask

    /**
     * This method removes the task object from the task list. If the task object is registered as a STANDALONE_TASK,
     * it will be unregistered so the standalone thread will be terminated.
     *
     * @param taskObj specifies the task object to be removed from the list.
     * @return true if the task object is removed successfully, false otherwise (e.g. no such task in the list).
     */
    private boolean removeTask(TaskObject taskObj)
    {
        if (taskObj.hasType(TaskType.STANDALONE_TASK))
        {
            //
            // Task contains the type STANDALONE_TASK, unregister it so that the task thread will terminate.
            //
            taskObj.unregisterTask(TaskType.STANDALONE_TASK);
        }

        return taskList.remove(taskObj);
    }   //removeTask

    /**
     * This method removes all tasks from the task list and terminate associated threads if any.
     */
    private void removeAllTask()
    {
        for (int i = taskList.size() - 1; i >= 0; i--)
        {
            removeTask(taskList.get(i));
        }
    }   //removeAllTask

    /**
     * This method is called at the end of the robot program (FtcOpMode in FTC or FrcRobotBase in FRC) to shut down
     * Task Manager and terminate all threads if any.
     */
    public void shutdown()
    {
        if (inputThread != null)
        {
            inputThread.terminateTask();
            inputThread = null;
        }

        if (outputThread != null)
        {
            outputThread.terminateTask();
            outputThread = null;
        }

        removeAllTask();
    }   //shutdown

    /**
     * This method is called by the main robot thread to enumerate the task list and calls all the tasks that matches
     * the given task type.
     *
     * @param type specifies the task type to be executed.
     * @param mode specifies the robot run mode.
     */
    public void executeTaskType(TaskType type, TrcRobot.RunMode mode)
    {
        final String funcName = "executeTaskType";
        //
        // Traverse the list backward because we are removing task objects from the list on STOP_TASK.
        // This way the list order won't be messed up.
        //
        for (int i = 0; i < taskList.size(); i++)
        {
            TaskObject taskObj = taskList.get(i);
            if (taskObj.hasType(type))
            {
                Task task = taskObj.getTask();
                long startNanoTime = TrcUtil.getCurrentTimeNanos();

                switch (type)
                {
                    case START_TASK:
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Executing StartTask %s", taskObj);
                        }
                        task.runTask(TaskType.START_TASK, mode);
                        break;

                    case STOP_TASK:
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Executing StopTask %s", taskObj);
                        }
                        task.runTask(TaskType.STOP_TASK, mode);
                        break;

                    case PREPERIODIC_TASK:
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Executing PrePeriodicTask %s", taskObj);
                        }
                        task.runTask(TaskType.PREPERIODIC_TASK, mode);
                        break;

                    case POSTPERIODIC_TASK:
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Executing PostPeriodicTask %s", taskObj);
                        }
                        task.runTask(TaskType.POSTPERIODIC_TASK, mode);
                        break;

                    case PRECONTINUOUS_TASK:
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Executing PreContinuousTask %s", taskObj);
                        }
                        task.runTask(TaskType.PRECONTINUOUS_TASK, mode);
                        break;

                    case POSTCONTINUOUS_TASK:
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Executing PostContinuousTask %s", taskObj);
                        }
                        task.runTask(TaskType.POSTCONTINUOUS_TASK, mode);
                        break;

                    case INPUT_TASK:
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Executing InputTask %s", taskObj);
                        }
                        task.runTask(TaskType.INPUT_TASK, mode);
                        break;

                    case OUTPUT_TASK:
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Executing OutputTask %s", taskObj);
                        }
                        task.runTask(TaskType.OUTPUT_TASK, mode);
                        break;

                    default:
                        break;
                }

                long elapsedTime = TrcUtil.getCurrentTimeNanos() - startNanoTime;
                taskObj.taskTotalNanoTimes[type.value] += elapsedTime;
                taskObj.taskTimeSlotCounts[type.value]++;

                if (debugEnabled)
                {
                    if (elapsedTime > taskNanoTimeThreshold)
                    {
                        dbgTrace.traceWarn(funcName, "%s.%s takes too long (%.3f)",
                            taskObj.taskName, type, elapsedTime/1000000000.0);
                    }
                }
            }
        }
    }   //executeTaskType

    /**
     * This method runs the periodic input task.
     *
     * @param context specifies the context (not used).
     */
    private synchronized void inputTask(Object context)
    {
        executeTaskType(TaskType.INPUT_TASK, TrcRobot.getRunMode());
    }   //inputTask

    /**
     * This method runs the periodic output task.
     *
     * @param context specifies the context (not used).
     */
    private synchronized void outputTask(Object context)
    {
        executeTaskType(TaskType.OUTPUT_TASK, TrcRobot.getRunMode());
    }   //outputTask

    /**
     * This method prints the performance metrics of all tasks with the given tracer.
     *
     * @param tracer specifies the tracer to be used for printing the task performance metrics.
     */
    public void printTaskPerformanceMetrics(TrcDbgTrace tracer)
    {
        for (TaskObject taskObj: taskList)
        {
            tracer.traceInfo(
                    "TaskPerformance",
                    "%16s: Start=%.6f, Stop=%.6f, PrePeriodic=%.6f, PostPeriodic=%.6f, " +
                    "PreContinuous=%.6f, PostContinous=%.6f, Standalone=%.6f, Input=%.6f, Output=%.6f",
                    taskObj.taskName,
                    (double)taskObj.taskTotalNanoTimes[TaskType.START_TASK.value]/
                            taskObj.taskTimeSlotCounts[TaskType.START_TASK.value]/1000000000,
                    (double)taskObj.taskTotalNanoTimes[TaskType.STOP_TASK.value]/
                            taskObj.taskTimeSlotCounts[TaskType.STOP_TASK.value]/1000000000,
                    (double)taskObj.taskTotalNanoTimes[TaskType.PREPERIODIC_TASK.value]/
                            taskObj.taskTimeSlotCounts[TaskType.PREPERIODIC_TASK.value]/1000000000,
                    (double)taskObj.taskTotalNanoTimes[TaskType.POSTPERIODIC_TASK.value]/
                            taskObj.taskTimeSlotCounts[TaskType.POSTPERIODIC_TASK.value]/1000000000,
                    (double)taskObj.taskTotalNanoTimes[TaskType.PRECONTINUOUS_TASK.value]/
                            taskObj.taskTimeSlotCounts[TaskType.PRECONTINUOUS_TASK.value]/1000000000,
                    (double)taskObj.taskTotalNanoTimes[TaskType.POSTCONTINUOUS_TASK.value]/
                            taskObj.taskTimeSlotCounts[TaskType.POSTCONTINUOUS_TASK.value]/1000000000,
                    (double)taskObj.taskTotalNanoTimes[TaskType.STANDALONE_TASK.value]/
                            taskObj.taskTimeSlotCounts[TaskType.STANDALONE_TASK.value]/1000000000,
                    (double)taskObj.taskTotalNanoTimes[TaskType.INPUT_TASK.value]/
                            taskObj.taskTimeSlotCounts[TaskType.INPUT_TASK.value]/1000000000,
                    (double)taskObj.taskTotalNanoTimes[TaskType.OUTPUT_TASK.value]/
                            taskObj.taskTimeSlotCounts[TaskType.OUTPUT_TASK.value]/1000000000);
        }
    }   //printTaskPerformanceMetrics

}   //class TaskMgr
