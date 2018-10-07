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
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import java.util.Arrays;

import hallib.HalDashboard;
import trclib.TrcDbgTrace;
import trclib.TrcSerialBusDevice;

/**
 * This class implements an FTC wrapper to the I2cDeviceSynch class that provides asynchronous read/write access
 * to the rawDevice by using a separate thread processing a request queue.
 */
public class FtcI2cDeviceAsync extends TrcSerialBusDevice
{
    private static final String moduleName = "FtcI2cDeviceAsync";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private FtcRawI2cDeviceSynch rawDevice;
    private I2cDeviceSynch deviceClient;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param i2cAddress specifies the I2C address of the rawDevice.
     * @param addressIs7Bit specifies true if the I2C address is a 7-bit address, false if it is 8-bit.
     */
    public FtcI2cDeviceAsync(HardwareMap hardwareMap, String instanceName, int i2cAddress, boolean addressIs7Bit)
    {
        super(instanceName);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(
                    moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        rawDevice = hardwareMap.get(FtcRawI2cDeviceSynch.class, instanceName);
        rawDevice.setI2cAddress(i2cAddress, addressIs7Bit);
        deviceClient = rawDevice.getDeviceClient();
    }   //FtcI2cDeviceAsync

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param i2cAddress specifies the I2C address of the rawDevice.
     * @param addressIs7Bit specifies true if the I2C address is a 7-bit address, false if it is 8-bit.
     */
    public FtcI2cDeviceAsync(String instanceName, int i2cAddress, boolean addressIs7Bit)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, i2cAddress, addressIs7Bit);
    }   //FtcI2cDeviceAsync

    /**
     * This method sets the rawDevice manufacturer and name.
     *
     * @param manufacturer specifies rawDevice manufacturer.
     * @param deviceName specifies rawDevice name.
     */
    public void setDeviceInfo(HardwareDevice.Manufacturer manufacturer, String deviceName)
    {
        final String funcName = "setDeviceInfo";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                    "manu=%s,name=%s", manufacturer, deviceName);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        rawDevice.setDeviceInfo(manufacturer, deviceName);
    }   //setDeviceInfo

    /**
     * This method check if the I2C rawDevice is enabled.
     *
     * @return true if the rawDevice state indicates it is enabled, false otherwise.
     */
    public boolean isDeviceEnabled()
    {
        final String funcName = "isDeviceEnabled";
        boolean enabled = deviceClient.isEngaged();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(enabled));
        }

        return enabled;
    }   //isDeviceEnabled

    /**
     * This method is called to enable/disable the I2C rawDevice so that it will not unnecessarily bog down the I2C bus
     * bandwidth if it is not needed.
     *
     * @param enabled specifies true to enable rawDevice, false otherwise.
     */
    public void setDeviceEnabled(boolean enabled)
    {
        final String funcName = "setDeviceEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (enabled)
        {
            deviceClient.engage();
            setTaskEnabled(true);
        }
        else
        {
            setTaskEnabled(false);
            deviceClient.disengage();
        }
    }   //setDeviceEnabled

    /**
     * This method is doing an asynchronous write to the rawDevice with the specified starting address and data of the
     * register block.
     *
     * @param startAddress specifies the starting register to read from.
     * @param data specifies the data to write to the rawDevice.
     */
    public void asyncWrite(int startAddress, byte[] data)
    {
        final String funcName = "asyncWrite";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "start=0x%02x,data=%s", startAddress, Arrays.toString(data));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        rawDevice.writeData(startAddress, data, false);
    }   //asyncWrite

    /**
     * This method sends a byte command to the rawDevice.
     *
     * @param regAddress specifies the register address to write to.
     * @param command specifies the command byte.
     * @param waitForCompletion specifies true to wait for write completion.
     */
    public void sendByteCommand(int regAddress, byte command, boolean waitForCompletion)
    {
        final String funcName = "sendByteCommand";
        byte[] data = new byte[1];

        data[0] = command;
        rawDevice.writeData(regAddress, data, waitForCompletion);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "command=%x", command);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //sendByteCommand

    /**
     * This method sends a 16-bit command to the rawDevice.
     *
     * @param regAddress specifies the register address to write to.
     * @param command specifies the 16-bit command.
     * @param waitForCompletion specifies true to wait for write completion.
     */
    public void sendWordCommand(int regAddress, short command, boolean waitForCompletion)
    {
        final String funcName = "sendWordCommand";
        byte[] data = new byte[2];

        data[0] = (byte)(command & 0xff);
        data[1] = (byte)(command >> 8);
        rawDevice.writeData(regAddress, data, waitForCompletion);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "command=%x", command);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //sendWordCommand

    //
    // Implements TrcSerialBusDevice abstract methods.
    //

    /**
     * This method is called to read data from the rawDevice synchronously with the specified length.
     *
     * @param address specifies the data address if any, can be -1 if no address is required.
     * @param length specifies the number of bytes to read.
     * @return a byte array containing the data read.
     */
    @Override
    public byte[] readData(int address, int length)
    {
        final String funcName = "readData";
        byte[] data = rawDevice.readData(address, length);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "start=0x%02x,len=%d", address, length);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Arrays.toString(data));
        }
        HalDashboard.getInstance().displayPrintf(4, "read(%d,%d):%s", address, length, Arrays.toString(data));

        return data;
    }   //readData

    /**
     * This method is called to write data to the rawDevice synchronously with the specified data buffer and length.
     *
     * @param address specifies the data address if any, can be -1 if no address is required.
     * @param buffer specifies the buffer containing the data to be written to the rawDevice.
     * @param length specifies the number of bytes to write.
     * @return number of bytes written.
     */
    @Override
    public int writeData(int address, byte[] buffer, int length)
    {
        final String funcName = "writeData";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                    "start=0x%02x,data=%s", address, Arrays.toString(buffer));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        rawDevice.writeData(address, buffer, true);
        HalDashboard.getInstance().displayPrintf(5, "write(%d,%d):%s", address, length, Arrays.toString(buffer));
        return length;
    }   //writeData

}   //class FtcI2cDeviceAsync
