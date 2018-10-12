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

package team3543;

import android.speech.tts.TextToSpeech;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import common.PixyVision;
import common.Robot;
import common.VuforiaVision;
import ftclib.FtcDcMotor;
import trclib.TrcMecanumDriveBase;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcRobot;

public class Robot3543 extends Robot
{
    static final String ROBOT_NAME = "Robot3543";
    static final boolean USE_VUFORIA = false;
    static final boolean USE_PIXY = true;
    //
    // Vision subsystems.
    //
    VuforiaVision vuforiaVision = null;
    RelicRecoveryVuMark prevVuMark = null;
    PixyVision pixyVision = null;
    //
    // DriveBase subsystem.
    //
    FtcDcMotor leftFrontWheel = null;
    FtcDcMotor rightFrontWheel = null;
    FtcDcMotor leftRearWheel = null;
    FtcDcMotor rightRearWheel = null;

    TrcPidController encoderXPidCtrl = null;
    TrcPidController encoderYPidCtrl = null;
    TrcPidController gyroPidCtrl = null;
    TrcPidDrive pidDrive = null;
    //
    // Other subsystems.
    //
    Elevator3543 elevator;
    TeamMarkerDeployer3543 teamMarkerDeployer;
    MineralSweeper3543 mineralSweeper;

    public Robot3543(TrcRobot.RunMode runMode)
    {
        super(runMode);
        //
        // Initialize global objects.
        //
        moduleName = ROBOT_NAME;
        //
        // Initialize vision subsystems.
        //
        if (USE_VUFORIA)
        {
            int cameraViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                    "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
            vuforiaVision = new VuforiaVision(this, -1);//cameraViewId);
        }

        if (USE_PIXY)
        {
            pixyVision = new PixyVision("pixy", this,
                    Robot3543Info.PIXY_ORIENTATION, Robot3543Info.PIXY_BRIGHTNESS);
        }
        //
        // Initialize DriveBase.
        //
        leftFrontWheel = new FtcDcMotor("lfWheel");
        rightFrontWheel = new FtcDcMotor("rfWheel");
        leftRearWheel = new FtcDcMotor("lrWheel");
        rightRearWheel = new FtcDcMotor("rrWheel");

        leftFrontWheel.motor.setMode(Robot3543Info.DRIVE_MOTOR_MODE);
        rightFrontWheel.motor.setMode(Robot3543Info.DRIVE_MOTOR_MODE);
        leftRearWheel.motor.setMode(Robot3543Info.DRIVE_MOTOR_MODE);
        rightRearWheel.motor.setMode(Robot3543Info.DRIVE_MOTOR_MODE);

        leftFrontWheel.setInverted(false);
        leftRearWheel.setInverted(false);
        rightFrontWheel.setInverted(true);
        rightRearWheel.setInverted(true);

        leftFrontWheel.setBrakeModeEnabled(true);
        leftRearWheel.setBrakeModeEnabled(true);
        rightFrontWheel.setBrakeModeEnabled(true);
        rightRearWheel.setBrakeModeEnabled(true);

        driveBase = new TrcMecanumDriveBase(leftFrontWheel, leftRearWheel, rightFrontWheel, rightRearWheel, gyro);
        driveBase.setPositionScales(Robot3543Info.ENCODER_X_INCHES_PER_COUNT, Robot3543Info.ENCODER_Y_INCHES_PER_COUNT);
        //
        // Initialize PID drive.
        //
        encoderXPidCtrl = new TrcPidController(
                "encoderXPidCtrl",
                new TrcPidController.PidCoefficients(
                        Robot3543Info.ENCODER_X_KP, Robot3543Info.ENCODER_X_KI, Robot3543Info.ENCODER_X_KD),
                Robot3543Info.ENCODER_X_TOLERANCE, driveBase::getXPosition);
        encoderYPidCtrl = new TrcPidController(
                "encoderYPidCtrl",
                new TrcPidController.PidCoefficients(
                        Robot3543Info.ENCODER_Y_KP, Robot3543Info.ENCODER_Y_KI, Robot3543Info.ENCODER_Y_KD),
                Robot3543Info.ENCODER_Y_TOLERANCE, driveBase::getYPosition);
        gyroPidCtrl = new TrcPidController(
                "gyroPidCtrl",
                new TrcPidController.PidCoefficients(
                        Robot3543Info.GYRO_KP, Robot3543Info.GYRO_KI, Robot3543Info.GYRO_KD),
                Robot3543Info.GYRO_TOLERANCE, driveBase::getHeading);
        gyroPidCtrl.setAbsoluteSetPoint(true);
        gyroPidCtrl.setOutputRange(-Robot3543Info.TURN_POWER_LIMIT, Robot3543Info.TURN_POWER_LIMIT);

        pidDrive = new TrcPidDrive("pidDrive", driveBase, encoderXPidCtrl, encoderYPidCtrl, gyroPidCtrl);
        pidDrive.setStallTimeout(Robot3543Info.PIDDRIVE_STALL_TIMEOUT);
        pidDrive.setBeep(androidTone);
        //
        // Initialize other subsystems.
        //
        elevator = new Elevator3543();
        teamMarkerDeployer = new TeamMarkerDeployer3543();
        mineralSweeper = new MineralSweeper3543();
        //
        // Tell the driver initialization is complete.
        //
        if (textToSpeech != null)
        {
            textToSpeech.speak("Init complete!", TextToSpeech.QUEUE_ADD, null);
        }
    }   //Robot

    @Override
    public void startMode(TrcRobot.RunMode runMode)
    {
        super.startMode(runMode);

        if (vuforiaVision != null)
        {
            vuforiaVision.setEnabled(true);
        }

        if (pixyVision != null)
        {
            pixyVision.setEnabled(true);
        }
        //
        // Reset all X, Y and heading values.
        //
        driveBase.resetOdometry();
    }   //startMode

    @Override
    public void stopMode(TrcRobot.RunMode runMode)
    {
        if (vuforiaVision != null)
        {
            vuforiaVision.setEnabled(false);
        }

        if (pixyVision != null)
        {
            pixyVision.setEnabled(false);
        }
    }   //stopMode

}   //class Robot3543
