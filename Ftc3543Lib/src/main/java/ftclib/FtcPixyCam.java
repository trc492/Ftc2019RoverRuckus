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

package ftclib;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;

import trclib.TrcDbgTrace;
import trclib.TrcPixyCam;

/**
 * This class implements a platform dependent pixy camera that is connected to an I2C bus.
 * It provides access to the last detected objects reported by the pixy camera asynchronously.
 */
public class FtcPixyCam extends TrcPixyCam
{
    private static final int DEF_I2C_ADDRESS = 0x54;
    private final FtcI2cDeviceAsync i2cDevice;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param devAddress specifies the I2C address of the device.
     * @param addressIs7Bit specifies true if the I2C address is a 7-bit address, false if it is 8-bit.
     */
    public FtcPixyCam(HardwareMap hardwareMap, String instanceName, int devAddress, boolean addressIs7Bit)
    {
        super(instanceName, false);

        i2cDevice = new FtcI2cDeviceAsync(hardwareMap, instanceName, devAddress, addressIs7Bit);
        i2cDevice.setDeviceInfo(HardwareDevice.Manufacturer.Other, "Pixy Camera v1");
        super.start();
    }   //FtcPixyCam

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param devAddress specifies the I2C address of the device.
     * @param addressIs7Bit specifies true if the I2C address is a 7-bit address, false if it is 8-bit.
     */
    public FtcPixyCam(String instanceName, int devAddress, boolean addressIs7Bit)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, devAddress, addressIs7Bit);
    }   //FtcPixyCam

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcPixyCam(String instanceName)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, DEF_I2C_ADDRESS, true);
    }   //FtcPixyCam

    /**
     * This method checks if the pixy camera is enabled.
     *
     * @return true if pixy camera is enabled, false otherwise.
     */
    public boolean isEnabled()
    {
        final String funcName = "isEnabled";
        boolean enabled = i2cDevice.isDeviceEnabled();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%b", enabled);
        }

        return enabled;
    }   //isEnable

    /**
     * This method enables/disables the pixy camera.
     *
     * @param enabled specifies true to enable pixy camera, false to disable.
     */
    public void setEnabled(boolean enabled)
    {
        final String funcName = "setEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enanbled=%b", enabled);
        }

        i2cDevice.setDeviceEnabled(enabled);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setEnabled

    /**
     * Indicates whether the background task encountered a problem and terminated unexpectedly.
     *
     * @return true if the background task has terminated unexpectedly, false otherwise.
     */
    public boolean isTaskTerminatedAbnormally()
    {
        return i2cDevice.isTaskTerminatedAbnormally();
    }   //isTaskTerminatedAbnormally

    //
    // Implements TrcPixyCam abstract methods.
    //

    /**
     * This method issues an asynchronous read of the specified number of bytes from the device.
     *
     * @param requestTag specifies the tag to identify the request. Can be null if none was provided.
     * @param length specifies the number of bytes to read.
     */
    @Override
    public void asyncReadData(RequestTag requestTag, int length)
    {
        final String funcName = "asyncReadData";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "tag=%s,length=%d",
                    requestTag != null? requestTag: "null", length);
        }

        i2cDevice.asyncRead(requestTag, length, null, this);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //asyncReadData

    /**
     * This method writes the data buffer to the device asynchronously.
     *
     * @param requestTag specifies the tag to identify the request. Can be null if none was provided.
     * @param data specifies the data buffer.
     */
    @Override
    public void asyncWriteData(RequestTag requestTag, byte[] data)
    {
        final String funcName = "asyncWriteData";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "tag=%s,data=%s,length=%d",
                    requestTag != null? requestTag: "null", Arrays.toString(data), data.length);
        }

        i2cDevice.asyncWrite(requestTag, data, data.length, null, null);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //asyncWriteData

}   //class FtcPixyCam
