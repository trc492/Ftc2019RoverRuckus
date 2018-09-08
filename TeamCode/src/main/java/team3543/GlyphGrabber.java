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
import trclib.TrcEnhancedServo;

public class GlyphGrabber
{
    private String instanceName;
    private FtcServo glyphLeftServo;
    private FtcServo glyphRightServo;
    private TrcEnhancedServo grabber;
    private double grabberPos = RobotInfo.GLYPH_GRABBER_START;

    public GlyphGrabber(String instanceName)
    {
        this.instanceName = instanceName;
        glyphLeftServo = new FtcServo("glyphLeftServo");
        glyphRightServo = new FtcServo("glyphRightServo");
        glyphLeftServo.setInverted(false);
        glyphRightServo.setInverted(true);
        grabber = new TrcEnhancedServo("glyphGrabber", glyphLeftServo, glyphRightServo);
    }   //GlyphGrabber

    public double getPosition()
    {
        return grabber.getPosition();
    }   //getPosition

    public void setPosition(double position)
    {
        grabberPos = position;
        grabber.setPosition(position);
    }   //setPosition

    public void open()
    {
        setPosition(grabberPos == RobotInfo.GLYPH_GRABBER_OPEN?
                RobotInfo.GLYPH_GRABBER_START: RobotInfo.GLYPH_GRABBER_OPEN);
    }   //open

    public void close()
    {
        setPosition(grabberPos == RobotInfo.GLYPH_GRABBER_CLOSE?
                RobotInfo.GLYPH_GRABBER_COLLAPSE: RobotInfo.GLYPH_GRABBER_CLOSE);
    }   //close

}   //class GlyphGrabber
