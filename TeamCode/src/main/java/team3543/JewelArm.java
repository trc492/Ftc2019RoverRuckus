/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
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

package team3543;

import ftclib.FtcServo;

public class JewelArm
{

    private String instanceName;
    private FtcServo verticalServo;
    private FtcServo horizontalServo;
    private boolean armExtended = false;

    /**
     * Constructor: Create an instance of the object  .
     */
    public JewelArm(String instanceName)
    {
        this.instanceName = instanceName;
        verticalServo = new FtcServo(instanceName + "VerticalServo");
        horizontalServo = new FtcServo(instanceName + "HorizontalServo");
    }   //JewelArm

    public String toString()
    {
        return instanceName;
    }

    public void setExtended(boolean extended)
    {
        armExtended = extended;
        verticalServo.setPosition(extended? RobotInfo.JEWEL_ARM_EXTENDED: RobotInfo.JEWEL_ARM_RETRACTED);
    }

    public void setSweepPosition(double pos)
    {
        horizontalServo.setPosition(pos);
    }

}   //class JewelArm
