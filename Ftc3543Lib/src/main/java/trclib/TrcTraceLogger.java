/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.concurrent.ConcurrentLinkedQueue;

public class TrcTraceLogger
{
    private String traceLogName;
    private PrintStream traceLog;
    private volatile boolean traceLogEnabled;
    private ConcurrentLinkedQueue<String> msgQueue;
    private TrcDbgTrace perfTracer = null;
    private double totalTime = 0.0;
    private int totalMessages = 0;

    /**
     * Constructor: Opens a log file for writing all the trace messages to it.
     *
     * @param traceLogName specifies the full trace log file path name.
     * @throws FileNotFoundException failed to open log file.
     */
    public TrcTraceLogger(final String traceLogName) throws FileNotFoundException
    {
        this.traceLogName = traceLogName;
        traceLog = new PrintStream(new File(traceLogName));
        traceLogEnabled = false;
        msgQueue = new ConcurrentLinkedQueue<>();

        TrcTaskMgr.TaskObject loggerTaskObj = TrcTaskMgr.getInstance().createTask(
                "loggerTask." + traceLogName, this::loggerTask);
        loggerTaskObj.registerTask(TrcTaskMgr.TaskType.STANDALONE_TASK);
    }   //TrcTraceLogger

    /**
     * This method returns the trace log name.
     *
     * @return trace log name.
     */
    public String toString()
    {
        return traceLogName;
    }   //toString

    /**
     * This method closes the trace log file. If newName is not null, the log will be renamed to the new name.
     *
     * @param newName specifies the new log file name, null if none given.
     */
    public synchronized void close(String newName)
    {
        if (traceLog != null)
        {
            if (newName != null)
            {
                try
                {
                    String path = traceLogName.substring(0, traceLogName.lastIndexOf(File.separatorChar) + 1);
                    String newFile = path + TrcUtil.getTimestamp() + "!" + newName + ".log";
                    traceLogEnabled = true;
                    traceLog.close();
                    File file = new File(traceLogName);
                    file.renameTo(new File(newFile));
                }
                catch(Exception e)
                {
                    // We failed to rename the file, close the log anyway.
                    traceLog.close();
                }
            }
            else
            {
                traceLog.close();
            }

            traceLog = null;
            traceLogName = null;
            traceLogEnabled = false;
        }
    }   //closeTraceLog

    /**
     * This method enables/disables performance report.
     *
     * @param tracer specifies the tracer to be used for performance tracing, can be null to disable performance
     *               tracing.
     */
    public synchronized void setPerformanceTracer(TrcDbgTrace tracer)
    {
        perfTracer = tracer;
    }   //setPerformanceTracer

    /**
     * This method enables/disables the trace log.
     *
     * @param enabled specifies true to enable trace log, false otherwise.
     */
    public synchronized void setEnabled(boolean enabled)
    {
        traceLogEnabled = enabled;
    }   //setEnabled

    /**
     * This method checks if the trace log is enabled.
     *
     * @return true if trace log is enabled, false if disabled.
     */
    public synchronized boolean isEnabled()
    {
        return traceLogEnabled;
    }   //isEnabled

    /**
     * This method is called to log a message to the log file.
     *
     * @param msg specifies the message to be logged.
     */
    public synchronized void logMessage(String msg)
    {
        if (traceLogEnabled)
        {
            msgQueue.add(msg);
        }
    }   //logMessage

    /**
     * This method is called periodically to process the logger message queue. Every time this task is run, it will
     * empty the message queue into the log file and flush on each write.
     *
     * @param taskType specifies the task type (not used).
     * @param runMode specifies the competition run mode (not used).
     */
    private synchronized void loggerTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "loggerTask";

        for (;;)
        {
            String msg = msgQueue.poll();

            if (msg != null)
            {
                double startTime = TrcUtil.getCurrentTime();
                traceLog.print(msg + "\r\n");
                traceLog.flush();
                double elapsedTime = TrcUtil.getCurrentTime() - startTime;
                totalTime += elapsedTime;
                totalMessages++;
                //
                // Make sure we don't recursively log the performance message itself.
                //
                if (perfTracer != null && !msg.startsWith(perfTracer.toString() + "." + funcName))
                {
                    perfTracer.traceInfo(funcName, "Average message log time = %.3f msec",
                            totalTime/totalMessages);
                }
            }
            else
            {
                break;
            }
        }
    }   //loggerTask

}   //class TrcTraceLogger
