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

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import java.util.Arrays;

/**
 * This class implements a platform dependent I2C Device that provides raw synchronous read/write access to the
 * device. For efficiency, it provides a buffered read mode that uses the FTC I2C ReadWindow interface to read
 * the maximum allowable number of bytes into a buffer. Then it will return data from this buffer FIFO and read
 * the next buffer if necessary.
 */
@I2cDeviceType
@DeviceProperties(name = "Raw I2C Device Synch", description = "Raw I2C Device Synch", xmlTag = "RawI2cDevice")
public class FtcRawI2cDeviceSynch extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    private final I2cDeviceSynch device;
    private Manufacturer manufacturer = Manufacturer.Other;
    private String deviceName = "Raw I2c Device";
    private int startReg = 0;
    private int cbReg = 0;
    private byte[] buffer = null;
    private int buffIndex = 0;

    /**
     * Constructor: Creates an instance of the object. This is intended to be called by the FTC SDK and the instance
     * will be available by calling hardwareMap.get(FtcRawI2cDeviceSynch.class, instanceName).
     *
     * @param device specifies the synchronous I2C device.
     */
    public FtcRawI2cDeviceSynch(I2cDeviceSynch device)
    {
        super(device, true);

        this.device = device;
        super.registerArmingStateCallback(false);
    }   //FtcRawI2cDeviceSynch

    /**
     * This method sets the I2C address of the device.
     *
     * @param i2cAddress specifies the I2C address.
     * @param addressIs7Bit specifies true if it is a 7-bit address, false if it is an 8-bit address.
     */
    public void setI2cAddress(int i2cAddress, boolean addressIs7Bit)
    {
        device.setI2cAddress(addressIs7Bit? I2cAddr.create7bit(i2cAddress): I2cAddr.create8bit(i2cAddress));
    }   //setI2cAddress

    /**
     * This method sets the device manufacturer and name.
     *
     * @param manufacturer specifies device manufacturer.
     * @param deviceName specifies device name.
     */
    public void setDeviceInfo(Manufacturer manufacturer, String deviceName)
    {
        this.manufacturer = manufacturer;
        this.deviceName = deviceName;
    }   //setDeviceInfo

    /**
     * This method sets the read window for reading a number of registers all at once into a data buffer. This is
     * primarily setting up for the readBufferedData method. If there is an existing buffer with data in it, this
     * call will blow that away and resets the index.
     *
     * @param startReg specifies the start register number.
     * @param cbReg specifies the number of bytes to be read into the buffer.
     */
    public void setBufferedReadWindow(int startReg, int cbReg)
    {
        this.startReg = startReg;
        this.cbReg = cbReg;
        this.buffer = null;
        this.buffIndex = 0;
    }   //setBufferedReadWindow

    /**
     * This method reads the specified number of bytes from the prefetched buffer. The I2cDeviceSynch
     * class implements a read window buffering a number of bytes. This method will return the specified bytes
     * from this buffer. If the data in the buffer runs out, it will read in the next buffer. If there is no more
     * data from the device, it will just return the data read so far or if there was none, null is returned.
     *
     * @param len specifies the number of bytes to read from the device.
     * @return the specified number of the bytes from the device.
     */
    public byte[] readBufferedData(int len)
    {
        byte[] data = new byte[len];
        int dataIndex = 0;

        while (dataIndex < len)
        {
            if (buffer != null && buffIndex < buffer.length)
            {
                data[dataIndex] = buffer[buffIndex];
                dataIndex++;
                buffIndex++;
            }
            else if (cbReg > 0)
            {
                buffer = device.read(startReg, cbReg);
                buffIndex = 0;
                if (buffer == null)
                {
                    //
                    // There is no more data from the device.
                    //
                    if (dataIndex > 0)
                    {
                        //
                        // Some data have been read but not enough to satisfy the whole amount being asked.
                        // Reallocate a smaller buffer and return it.
                        //
                        data = Arrays.copyOf(data, dataIndex);
                    }
                    else
                    {
                        data = null;
                    }
                    break;
                }
            }
            else
            {
                //
                // There is no readWindow defined yet. So just read the data directly from the device.
                //
                data = device.read(0, len);
                break;
            }
        }

        return data;
    }   //readBufferedData

    /**
     * This method reads the specified number of bytes from the device start at the specified register.
     *
     * @param startReg specifies the start register to read.
     * @param len specifies the number of bytes to read.
     *
     * @return data read.
     */
    public byte[] readData(int startReg, int len)
    {
        return device.read(startReg, len);
    }   //readData

    /**
     * This method writes the specified number of bytes to the device via theI2cDeviceSynch.
     *
     * @param startReg specifies the start register to read.
     * @param data specifies the number of bytes to read from the device.
     * @param waitForCompletion specifies true to wait for write completion.
     */
    public void writeData(int startReg, byte[] data, boolean waitForCompletion)
    {
        device.write(startReg, data, waitForCompletion? I2cWaitControl.WRITTEN: I2cWaitControl.ATOMIC);
    }   //writeData

    //
    // Implements abstract methods required by I2cDeviceSynchDevice.
    //

    /**
     * This method is called to initialize the device. There is nothing to do in this case.
     *
     * @return true.
     */
    @Override
    protected synchronized boolean doInitialize()
    {
        return true;
    }   //doInitialize

    /**
     * This method returns the manufacture of the device.
     *
     * @return manufacturer.
     */
    @Override
    public Manufacturer getManufacturer()
    {
        return manufacturer;
    }   //getManufacturer

    /**
     * Returns a string suitable for display to the user as to the type of device.
     * Note that this is a device-type-specific name; it has nothing to do with the
     * name by which a user might have configured the device in a robot configuration.
     *
     * @return device name string.
     */
    @Override
    public String getDeviceName()
    {
        return deviceName;
    }   //getDeviceName

}   //class FtcRawI2cDeviceSynch
