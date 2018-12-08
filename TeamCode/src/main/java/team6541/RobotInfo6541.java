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

import common.RobotInfo;

class RobotInfo6541 extends RobotInfo
{
    //
    // DriveBase subsystem.
    //
    static final DcMotor.RunMode DRIVE_MOTOR_MODE               = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    static final double TURN_POWER_LIMIT                        = 0.5;
    //
    // 2018-11-02: Kp=0.1, Ki=0.0, Kd=0.0, Scale=0.0152133737
    // 2018-11-29: Kp=0.1, Ki=0.0, Kd=0.01, Scale=0.0150461720101369
    //
    static final double ENCODER_Y_KP                            = 0.1;
    static final double ENCODER_Y_KI                            = 0.0;
    static final double ENCODER_Y_KD                            = 0.01;
    static final double ENCODER_Y_TOLERANCE                     = 1.0;
    static final double ENCODER_Y_INCHES_PER_COUNT              = 0.0150461720101369;
    //
    // 2018-11-02: Kp=0.025, Ki=0.0, Kd=0.002
    // 2018-11-29: Kp=0.022, Ki=0.0, Kd=0.0025
    //
    static final double GYRO_KP                                 = 0.02;
    static final double GYRO_KI                                 = 0.0;
    static final double GYRO_KD                                 = 0.00275;
    static final double GYRO_TOLERANCE                          = 2.0;

    static final double PIDDRIVE_STALL_TIMEOUT                  = 0.5;     //in seconds.
    //
    // Elevator subsystem.
    // 2018-10-27: Kp=3.0, Ki=0.0, Kd=0.0, Scale=0.001219829114490629
    //
    public static final double ELEVATOR_INCHES_PER_COUNT        = 0.001219829114490629;
    public static final double ELEVATOR_ZERO_OFFSET             = 16.75;
    public static final double ELEVATOR_KP                      = 3.0;
    public static final double ELEVATOR_KI                      = 0.0;
    public static final double ELEVATOR_KD                      = 0.0;
    public static final double ELEVATOR_TOLERANCE               = 0.2;
    public static final double ELEVATOR_MIN_HEIGHT              = ELEVATOR_ZERO_OFFSET - 0.1;
    public static final double ELEVATOR_MAX_HEIGHT              = 22.75;
    public static final double ELEVATOR_HANGING_HEIGHT          = 21.7;
    public static final double ELEVATOR_CAL_POWER               = 0.3;
    //
    // Other subsystems.
    //
    public static final double DEPLOYER_OPEN_POSITION           = 0.2;
    public static final double DEPLOYER_CLOSE_POSITION          = 0.8;
    public static final double HANGING_HOOK_OPEN_POSITION       = 0.2;
    public static final double HANGING_HOOK_CLOSE_POSITION      = 0.7;
    public static final double MINERAL_SCOOPER_EXTEND_POSITION  = 0.0;
    public static final double MINERAL_SCOOPER_RETRACT_POSITION = 0.7;
    //
    // Vision subsystem.
    //
    public static final double SIDE_MINERAL_ANGLE               = 30.0;

}   //class RobotInfo6541
