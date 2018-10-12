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

package team6541;

import com.qualcomm.robotcore.hardware.DcMotor;

import common.PixyVision;

class Robot6541Info
{
    //
    // DriveBase subsystem.
    //
    static final DcMotor.RunMode DRIVE_MOTOR_MODE       = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    static final double TURN_POWER_LIMIT                = 0.5;

    static final double ENCODER_X_KP                    = 0.15;
    static final double ENCODER_X_KI                    = 0.0;
    static final double ENCODER_X_KD                    = 0.015;
    static final double ENCODER_X_TOLERANCE             = 2.0;
    static final double ENCODER_X_INCHES_PER_COUNT      = 63.0/4403.2;

    static final double ENCODER_Y_KP                    = 0.02;
    static final double ENCODER_Y_KI                    = 0.0;
    static final double ENCODER_Y_KD                    = 0.0022;
    static final double ENCODER_Y_TOLERANCE             = 1.0;
    static final double ENCODER_Y_INCHES_PER_COUNT      = 68.0/4100.5;

    static final double GYRO_KP                         = 0.018;
    static final double GYRO_KI                         = 0.0;
    static final double GYRO_KD                         = 0.002;
    static final double GYRO_TOLERANCE                  = 1.0;

    static final double PIDDRIVE_STALL_TIMEOUT          = 0.25;     //in msec.
    //
    // Elevator subsystem
    //
    public static final double ELEVATOR_INCHES_PER_COUNT = 5.625/8498;
    public static final double ELEVATOR_ZERO_OFFSET = 15.8;
    public static final double ELEVATOR_KP = 3.0;
    public static final double ELEVATOR_KI = 0.0;
    public static final double ELEVATOR_KD = 0.0;
    public static final double ELEVATOR_TOLERANCE = 0.2;
    public static final double ELEVATOR_MIN_HEIGHT = ELEVATOR_ZERO_OFFSET - 0.1;
    public static final double ELEVATOR_MAX_HEIGHT = 24.0;
    public static final double ELEVATOR_CAL_POWER = 0.3;
    //
    // PixyVision subsystem.
    //
    public static final int PIXY_GOLD_MINERAL_SIGNATURE         = 1;
    public static final int PIXY_SILVER_MINERAL_SIGNATURE       = 2;
    public static final int PIXY_TEAM_MARKER1_SIGNATURE         = 3;
    public static final int PIXY_TEAM_MARKER2_SIGNATURE         = 4;
    public static final int PIXY_BRIGHTNESS                     = 80;
    public static final double PIXY_CAM_OFFSET                  = 8.0;
    public static final PixyVision.Orientation PIXY_ORIENTATION = PixyVision.Orientation.NORMAL_LANDSCAPE;
    //
    // Hanging hook subsystem
    //

    public static final double HANGING_HOOK_OPEN_POSITION       = 0.8;
    public static final double HANGING_HOOK_CLOSE_POSITION      = 0.2;
    public static final double DEPLOYER_OPEN_POSITION           = 0.8;
    public static final double DEPLOYER_CLOSE_POSITION          = 0.2;
    public static final double MINERAL_SWEEPER_OPEN_POSITION    = 0.8;
    public static final double MINERAL_SWEEPER_CLOSE_POSITION   = 0.2;
}   //class Robot6541Info
