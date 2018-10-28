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

package common;

public class RobotInfo
{
    //
    // Velocity controlled constants.
    //
    public static final double MOTOR_MAX_VELOCITY               = 3080.0;   //encoder counts per second
    public static final double MOTOR_KP                         = 0.75;
    public static final double MOTOR_KI                         = 0.0;
    public static final double MOTOR_KD                         = 0.0;
    //
    // PixyVision subsystem.
    //
    public static final int PIXY_GOLD_MINERAL_SIGNATURE         = 1;
    public static final int PIXY_SILVER_MINERAL_SIGNATURE       = 2;
    public static final int PIXY_TEAM_MARKER1_SIGNATURE         = 3;
    public static final int PIXY_TEAM_MARKER2_SIGNATURE         = 4;

}   //class RobotInfo
