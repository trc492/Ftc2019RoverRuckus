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

import trclib.TrcTaskMgr.TaskType;

/**
 * This class implements a platform independent motor controller. Typically, this class is extended by a platform
 * dependent motor controller class. Not all motor controllers are created equal. Some have more features than the
 * others. This class attempts to emulate some of the features in software. If the platform dependent motor controller
 * supports some features in hardware it should override the corresponding methods and call the hardware directly.
 * For some features that there is no software emulation, this class will throw an UnsupportedOperationException.
 * If the motor controller hardware support these features, the platform dependent class should override these methods
 * to provide the support in hardware.
 */
public abstract class TrcMotor implements TrcMotorController
{
    protected static final String moduleName = "TrcMotor";
    protected static final boolean debugEnabled = false;
    protected static final boolean tracingEnabled = false;
    protected static final boolean useGlobalTracer = false;
    protected static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    protected static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;

    private final String instanceName;
    private final TrcTaskMgr.TaskObject motorSpeedTaskObj;
    private final TrcTaskMgr.TaskObject motorStopTaskObj;
    private TrcDigitalTrigger digitalTrigger = null;
    private boolean speedTaskEnabled = false;
    private double speedSensorUnitsPerSec = 0.0;
    private double prevTime = 0.0;
    private double prevPos = 0.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcMotor(final String instanceName)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        TrcTaskMgr taskMgr = TrcTaskMgr.getInstance();
        motorSpeedTaskObj = taskMgr.createTask(instanceName + ".motorSpeedTask", this::motorSpeedTask);
        motorStopTaskObj = taskMgr.createTask(instanceName + ".motorSpeedTask", this::motorStopTask);
    }   //TrcMotor

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method creates a digital trigger on the given digital input sensor. It resets the position sensor
     * reading when the digital input is triggered.
     *
     * @param digitalInput specifies the digital input sensor that will trigger a position reset.
     */
    public void resetPositionOnDigitalInput(TrcDigitalInput digitalInput)
    {
        final String funcName = "resetPositionOnDigitalInput";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "digitalInput=%s", digitalInput);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        digitalTrigger = new TrcDigitalTrigger(instanceName, digitalInput, this::triggerEvent);
        digitalTrigger.setTaskEnabled(true);
    }   //resetPositionOnDigitalInput

    /**
     * This method enables/disables the task that monitors the motor speed. To determine the motor speed, the task
     * runs periodically and determines the delta encoder reading over delta time to calculate the speed. Since the
     * task takes up CPU cycle, it should not be enabled if the user doesn't need motor speed info.
     *
     * @param enabled specifies true to enable speed monitor task, disable otherwise.
     */
    public synchronized void setTaskEnabled(boolean enabled)
    {
        final String funcName = "setTaskEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", Boolean.toString(enabled));
        }

        speedTaskEnabled = enabled;
        if (enabled)
        {
            prevTime = TrcUtil.getCurrentTime();
            prevPos = getPosition();
            motorStopTaskObj.registerTask(TrcTaskMgr.TaskType.STOP_TASK);
            motorSpeedTaskObj.registerTask(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
        }
        else
        {
            motorStopTaskObj.unregisterTask(TrcTaskMgr.TaskType.STOP_TASK);
            motorSpeedTaskObj.unregisterTask(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setTaskEnabled

    /**
     * This method is called periodically to calculate he speed of the motor.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running.
     */
    public synchronized void motorSpeedTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "motorSpeedTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        double currTime = TrcUtil.getCurrentTime();
        double currPos = getPosition();
        speedSensorUnitsPerSec = (currPos - prevPos)/(currTime - prevTime);
        prevTime = currTime;
        prevPos = currPos;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //motorSpeedTask

    /**
     * This method is called before competition mode is about to end so we can stop the task.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running.
     */
    public void motorStopTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "motorStopTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        setTaskEnabled(false);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //motorStopTask

    //
    // Implements the TrcMotorController interface.
    //

    /**
     * This method returns the speed of the motor rotation. It keeps track of the rotation speed by using a periodic
     * task to monitor the position sensor value. If the motor controller has hardware monitoring speed, it should
     * override this method and access the hardware instead.
     *
     * @return motor speed in sensor units per second.
     */
    @Override
    public synchronized double getSpeed()
    {
        final String funcName = "getSpeed";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%.3f", speedSensorUnitsPerSec);
        }

        if (speedTaskEnabled)
        {
            return speedSensorUnitsPerSec;
        }
        else
        {
            throw new UnsupportedOperationException("MotorTask is not enabled.");
        }
    }   //getSpeed

    //
    // Implements TrcDigitalTrigger.TriggerHandler.
    //

    /**
     * This method is called when the digital input device has changed state.
     *
     * @param active specifies true if the digital device state is active, false otherwise.
     */
    public void triggerEvent(boolean active)
    {
        final String funcName = "triggerEvent";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.CALLBK,
                    "trigger=%s,active=%s", digitalTrigger, Boolean.toString(active));
        }

        resetPosition(false);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.CALLBK);
        }
    }   //triggerEvent

}   //class TrcMotor
