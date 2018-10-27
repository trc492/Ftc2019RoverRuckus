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

import common.MineralSweeper;
import common.Robot;
import common.RobotInfo;
import common.TeamMarkerDeployer;
import ftclib.FtcDcMotor;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcRobot;
import trclib.TrcSimpleDriveBase;

public class Robot6541 extends Robot
{
    static final String ROBOT_NAME = "Robot6541";
    //
    // Other subsystems.
    //
    public Elevator6541 elevator;

    public Robot6541(TrcRobot.RunMode runMode)
    {
        super(runMode);
        //
        // Initialize global objects.
        //
        moduleName = ROBOT_NAME;
        //
        // Initialize DriveBase.
        //
        leftFrontWheel = new FtcDcMotor("lfWheel");
        rightFrontWheel = new FtcDcMotor("rfWheel");
        leftRearWheel = new FtcDcMotor("lrWheel");
        rightRearWheel = new FtcDcMotor("rrWheel");

        leftFrontWheel.motor.setMode(Robot6541Info.DRIVE_MOTOR_MODE);
        rightFrontWheel.motor.setMode(Robot6541Info.DRIVE_MOTOR_MODE);
        leftRearWheel.motor.setMode(Robot6541Info.DRIVE_MOTOR_MODE);
        rightRearWheel.motor.setMode(Robot6541Info.DRIVE_MOTOR_MODE);

        if (USE_VELOCITY_CONTROL)
        {
            TrcPidController.PidCoefficients motorPidCoef = new TrcPidController.PidCoefficients(
                    RobotInfo.MOTOR_KP, RobotInfo.MOTOR_KI, RobotInfo.MOTOR_KD);
            leftFrontWheel.enableVelocityMode(RobotInfo.MOTOR_MAX_VELOCITY, motorPidCoef);
            rightFrontWheel.enableVelocityMode(RobotInfo.MOTOR_MAX_VELOCITY, motorPidCoef);
            leftRearWheel.enableVelocityMode(RobotInfo.MOTOR_MAX_VELOCITY, motorPidCoef);
            rightRearWheel.enableVelocityMode(RobotInfo.MOTOR_MAX_VELOCITY, motorPidCoef);
        }

        leftFrontWheel.setInverted(false);
        leftRearWheel.setInverted(false);
        rightFrontWheel.setInverted(false);
        rightRearWheel.setInverted(true);

        leftFrontWheel.setBrakeModeEnabled(true);
        leftRearWheel.setBrakeModeEnabled(true);
        rightFrontWheel.setBrakeModeEnabled(true);
        rightRearWheel.setBrakeModeEnabled(true);

        driveBase = new TrcSimpleDriveBase(leftFrontWheel, leftRearWheel, rightFrontWheel, rightRearWheel, gyro);
        driveBase.setPositionScales(Robot6541Info.ENCODER_X_INCHES_PER_COUNT, Robot6541Info.ENCODER_Y_INCHES_PER_COUNT);
        //
        // Initialize PID drive.
        //
        encoderXPidCtrl = new TrcPidController(
                "encoderXPidCtrl",
                new TrcPidController.PidCoefficients(
                        Robot6541Info.ENCODER_X_KP, Robot6541Info.ENCODER_X_KI, Robot6541Info.ENCODER_X_KD),
                Robot6541Info.ENCODER_X_TOLERANCE, driveBase::getXPosition);
        encoderYPidCtrl = new TrcPidController(
                "encoderYPidCtrl",
                new TrcPidController.PidCoefficients(
                        Robot6541Info.ENCODER_Y_KP, Robot6541Info.ENCODER_Y_KI, Robot6541Info.ENCODER_Y_KD),
                Robot6541Info.ENCODER_Y_TOLERANCE, driveBase::getYPosition);
        gyroPidCtrl = new TrcPidController(
                "gyroPidCtrl",
                new TrcPidController.PidCoefficients(
                        Robot6541Info.GYRO_KP, Robot6541Info.GYRO_KI, Robot6541Info.GYRO_KD),
                Robot6541Info.GYRO_TOLERANCE, this::getHeading);
        gyroPidCtrl.setAbsoluteSetPoint(true);
        gyroPidCtrl.setOutputRange(-Robot6541Info.TURN_POWER_LIMIT, Robot6541Info.TURN_POWER_LIMIT);

        pidDrive = new TrcPidDrive("pidDrive", driveBase, encoderXPidCtrl, encoderYPidCtrl, gyroPidCtrl);
        pidDrive.setStallTimeout(Robot6541Info.PIDDRIVE_STALL_TIMEOUT);
        pidDrive.setBeep(androidTone);
        //
        // Initialize other subsystems.
        //
        elevator = new Elevator6541();
        mineralSweeper = new MineralSweeper(
                Robot6541Info.MINERAL_SWEEPER_EXTEND_POSITION, Robot6541Info.MINERAL_SWEEPER_RETRACT_POSITION);
        teamMarkerDeployer = new TeamMarkerDeployer(
                Robot6541Info.DEPLOYER_OPEN_POSITION, Robot6541Info.DEPLOYER_CLOSE_POSITION);
        //
        // Tell the driver initialization is complete.
        //
        speak("Init complete!");
    }   //Robot

    @Override
    public void startMode(TrcRobot.RunMode runMode)
    {
        super.startMode(runMode);
    }   //startMode

    @Override
    public void stopMode(TrcRobot.RunMode runMode)
    {
        super.stopMode(runMode);
    }   //stopMode

}   //class Robot6541
