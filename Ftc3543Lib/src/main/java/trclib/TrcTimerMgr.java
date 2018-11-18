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
import java.util.HashMap;

/**
 * This class implements the TrcTimer manager that uses a single thread to monitor all TrcTimers.
 */
public class TrcTimerMgr
{
    private static final String moduleName = "TrcTimerMgr";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private static TrcTimerMgr instance = null;
    private final ArrayList<TrcTimer> timerList;
    private HashMap<TrcTimer, Double> securityKeyMap;
    private final TrcTaskMgr.TaskObject timerTaskObj;

    /**
     * Constructor: Creates an instance of the timer manager.
     */
    private TrcTimerMgr()
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        timerList = new ArrayList<>();
        securityKeyMap = new HashMap<>();
        timerTaskObj = TrcTaskMgr.getInstance().createTask(moduleName + ".timerTask", this::timerTask);
        //
        // Timer manager needs its own thread running as fast as it can to process all the timers. This ensures faster
        // timeout response.
        //
        timerTaskObj.registerTask(TrcTaskMgr.TaskType.STANDALONE_TASK, 10);
    }   //TrcTimerMgr

    /**
     * This method returns the instance of TrcTimerMgr. If this is the first time it's called, TrcTimerMgr is created.
     *
     * @return instance of TrcTimerMgr.
     */
    public static TrcTimerMgr getInstance()
    {
        if (instance == null)
        {
            instance = new TrcTimerMgr();
        }

        return instance;
    }   //getInstance

    /**
     * This method adds the timer to the timer list in the order of expiration.
     *
     * @param timer specifies the timer to be added to the list.
     * @param securityKey specifies the security key for identifying the caller.
     * @return position inserted in the list.
     */
    public synchronized int add(TrcTimer timer, double securityKey)
    {
        final String funcName = "add";
        double expiredTime = timer.getExpiredTime();
        int position = -1;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                    "timer=%s,securityKey=%f", timer, securityKey);
        }

        for (int i = 0; i < timerList.size(); i++)
        {
            if (expiredTime < timerList.get(i).getExpiredTime())
            {
                position = i;
                break;
            }
        }

        if (position == -1)
        {
            //
            // This must be the last in the list, add it to the end.
            //
            position = timerList.size();
        }

        timerList.add(position, timer);
        //
        // The final security key is the sum of the list position and the caller's identification key.
        //
        securityKeyMap.put(timer, position + securityKey);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", position);
            dbgTrace.traceInfo("add", "Adding timer %s: securityKey=%.3f->%d", timer, securityKey, position);
        }

        return position;
    }   //add

    /**
     * This method removes a timer from the list.
     *
     * @param timer specifies the timer to be removed.
     * @param securityKey specifies the security key identifying the owner of the timer to be removed.
     * @return true if the timer is removed, false otherwise.
     */
    public synchronized boolean remove(TrcTimer timer, double securityKey)
    {
        final String funcName = "remove";
        boolean success = false;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                    "timer=%s,securityKey=%f", timer, securityKey);
        }

        if (securityKey == securityKeyMap.get(timer))
        {
            success = timerList.remove(timer);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", success);
            dbgTrace.traceInfo(funcName, "timer=%s,securityKey=%f,timerSec=%f",
                    timer, securityKey, securityKeyMap.get(timer));
        }

        return success;
    }   //remove

    /**
     * This method runs periodically to check if the next timer in the list has expired. If so, the timer is removed
     * and the timer's expiration handler is called.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the current robot run mode.
     */
    private synchronized void timerTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "timerTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        TrcTimer timer = timerList.isEmpty()? null: timerList.get(0);
        if (timer != null)
        {
            if (TrcUtil.getCurrentTime() >= timer.getExpiredTime())
            {
                timerList.remove(0);
                timer.setExpired(securityKeyMap.get(timer));
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //timerTask

}   //class TrcTimerMgr
