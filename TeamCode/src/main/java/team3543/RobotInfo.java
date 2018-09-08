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

import com.qualcomm.robotcore.hardware.DcMotor;

class RobotInfo
{
    static final float MM_PER_INCH                      = 25.4f;

    //
    // Color sensor values.
    //
    static final double RED1_LOW_THRESHOLD              = 1.0;
    static final double RED1_HIGH_THRESHOLD             = 40.0;
    static final double BLUE_LOW_THRESHOLD              = 150.0;
    static final double BLUE_HIGH_THRESHOLD             = 220.0;
    static final double RED2_LOW_THRESHOLD              = 350.0;
    static final double RED2_HIGH_THRESHOLD             = 359.0;

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

//    static final double ANALOG_GYRO_SCALE               = 1.0136;
//    static final double ANALOG_GYRO_VOLT_PER_DEG_PER_SEC= 0.007;

    static final double GYRO_KP                         = 0.018;
    static final double GYRO_KI                         = 0.0;
    static final double GYRO_KD                         = 0.002;
    static final double GYRO_TOLERANCE                  = 1.0;

    static final double PIDDRIVE_STALL_TIMEOUT          = 0.25;     //in msec.

    /**
     * @author TRC Programming Team, Elaine Z., Victor D.
     *
     * === VARIABLE VISION_KP CHANGELOG BELOW ===
     * Tune 1: 1.0
     * Tune 4: Useless stuff here
     * Tune 5: 0.0125 (Best of Thursday 10/5/2017, 20:53:16)
     * === END VARIABLE VISION_KP CHANGELOG.. ===
     */
    static final double VISION_KP                       = 0.0125;
    static final double VISION_KI                       = 0.0;
    static final double VISION_KD                       = 0.0;
    static final double VISION_TOLERANCE                = 1.0;

    static final double RANGE_ERROR_THRESHOLD           = 50.0; // value should not jump 50 inches in one timeslice.
    static final double RANGE_X_KP                      = 0.12;
    static final double RANGE_X_KI                      = 0.0;
    static final double RANGE_X_KD                      = 0.012;
    static final double RANGE_X_TOLERANCE               = 0.5;

    //
    // Maxbotix ultrasonic sensor.
    //
    // According to the sensor spec, the sensor reports Vcc/512 volts per inch.
    // For some reason unknown to us, we are a factor of 2 off. So the reading seems to be correct if we use 1024
    // instead of 512.
    //
    static final double SONAR_INCHES_PER_VOLT           = (1024.0/3.3);
    static final double SONAR_ERROR_THRESHOLD           = 50.0; // value should not jump 50 inches in one timeslice.

    static final double SONAR_X_KP                      = 0.045;
    static final double SONAR_X_KI                      = 0.0;
    static final double SONAR_X_KD                      = 0.0;
    static final double SONAR_X_TOLERANCE               = 1.0;

    static final double SONAR_Y_KP                      = 0.0112;
    static final double SONAR_Y_KI                      = 0.0;
    static final double SONAR_Y_KD                      = 0.0;
    static final double SONAR_Y_TOLERANCE               = 1.0;
    //
    // GlyphGrabber subsystem.
    //
    static final double GLYPH_GRABBER_COLLAPSE          = 0.0;
    static final double GLYPH_GRABBER_CLOSE             = 0.2;
    static final double GLYPH_GRABBER_OPEN              = 0.4;
    static final double GLYPH_GRABBER_START             = 0.6;

    //
    // JewelBar subsystem.
    //
    static final double JEWEL_ARM_RETRACTED             = 0.1;
    static final double JEWEL_ARM_EXTENDED              = 0.75;
    static final double JEWEL_ARM_NEUTRAL               = 0.5;
    static final double JEWEL_ARM_FORWARD               = 0.0;
    static final double JEWEL_ARM_BACKWARD              = 1.0;
    //
    // GlyphElevator subsystem.
    //
    static final double ELEVATOR_INCHES_PER_COUNT       = 0.002822426329889;
    static final double ELEVATOR_KP                     = 0.3;
    static final double ELEVATOR_KI                     = 0.0;
    static final double ELEVATOR_KD                     = 0.0;
    static final double ELEVATOR_TOLERANCE              = 0.5;
    static final double ELEVATOR_MIN_HEIGHT             = 0.0;
    static final double ELEVATOR_MAX_HEIGHT             = 21.0;
    static final double ELEVATOR_MID_HEIGHT             = 7.0;
    static final double ELEVATOR_CAL_POWER              = 0.3;
    static final double ELEVATOR_SENSOR_ZERO_OFFSET     = -0.529;
    static final double ELEVATOR_SENSOR_SCALE           = 11.18586208869386;

    //
    // RelicArm subsystem.
    //
    static final double RELIC_GRABBER_CLOSE             = 0.0;
    static final double RELIC_GRABBER_OPEN              = 0.5;
    static final double RELIC_ELBOW_KP                  = 0.1;  //???
    static final double RELIC_ELBOW_KI                  = 0.0;
    static final double RELIC_ELBOW_KD                  = 0.0;
    static final double RELIC_ELBOW_TOLERANCE           = 2.0;
    static final double RELIC_ELBOW_MIN_POS             = -40.0;
    static final double RELIC_ELBOW_MAX_POS             = 200.0;    //???
    static final double RELIC_ELBOW_CAL_POWER           = 0.3;
    //
    // To counteract gravity, we need to add power compensation to the elbow motor.
    // We are using a NeveRest 60 motor. The performance spec of this motor is:
    // - Stall Torque: 593 oz-in
    // - Output counts per revolution of Output Shaft (cpr): 1680 Pulses
    // The greatest effort to hold the elbow is when the arm is horizontal.
    // Arm length = 19 inches.
    // Weight of arm at 19-inch from fulcrum = 21.16 oz.
    // Torque at fulcrum = Weight of arm * arm length = 21.16*19 = 402.04 oz-in.
    // Additional gear ratio = 2:1.
    // Torque at motor = 402.04/2 = 201.02 oz-in.
    // To maintain arm at horizontal position, we need to apply 201.02/593 = 0.339 of full power.
    // To calculate Degrees/EncoderCount, one revolution of the motor will give 1680 counts.
    // One revolution of the motor will yield 180-degree movement of the arm because of the 2:1 gear ratio.
    // Therefore, the degrees_per_count = 180.0/1680.0.
    // The arm is resting at 40 degrees below the horizontal position. A lower limit switch will clear the encoder
    // when the arm is at rest. So the scaled position read from the encoder should subtract 40 degrees from it.
    //
    static final double RELIC_ELBOW_DEGREES_PER_COUNT   = (180.0/1680.0);
    static final double RELIC_ELBOW_POS_OFFSET          = -40.0;
    static final double RELIC_ELBOW_LEVEL_MOTOR_POWER   = 0.339*1.0;

}   //class RobotInfo
