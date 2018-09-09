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

import android.speech.tts.TextToSpeech;
import android.widget.TextView;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import ftclib.FtcAndroidTone;
import ftclib.FtcBNO055Imu;
import ftclib.FtcDcMotor;
import ftclib.FtcMenu;
import ftclib.FtcOpMode;
import ftclib.FtcRobotBattery;
import hallib.HalDashboard;
import trclib.TrcDbgTrace;
import trclib.TrcGyro;
import trclib.TrcMecanumDriveBase;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcRobot;

public class Robot implements FtcMenu.MenuButtons
{
    static final boolean USE_SPEECH = true;
    static final boolean USE_VUFORIA = true;

    private static final String moduleName = "Robot";
    //
    // Global objects.
    //
    FtcOpMode opMode;
    HalDashboard dashboard;
    TrcDbgTrace tracer;
    FtcRobotBattery battery = null;
    FtcAndroidTone androidTone;
    TextToSpeech textToSpeech = null;
    //
    // Sensors.
    //
    FtcBNO055Imu imu = null;
    TrcGyro gyro = null;
    double targetHeading = 0.0;
    //
    // Vision subsystems.
    //
    VuforiaVision vuforiaVision = null;
    RelicRecoveryVuMark prevVuMark = null;
    //
    // DriveBase subsystem.
    //
    FtcDcMotor leftFrontWheel = null;
    FtcDcMotor rightFrontWheel = null;
    FtcDcMotor leftRearWheel = null;
    FtcDcMotor rightRearWheel = null;
    TrcMecanumDriveBase driveBase = null;

    TrcPidController encoderXPidCtrl = null;
    TrcPidController encoderYPidCtrl = null;
    TrcPidController gyroPidCtrl = null;
    TrcPidDrive pidDrive = null;
    //
    // Other subsystems.
    //

    public Robot(TrcRobot.RunMode runMode)
    {
        //
        // Initialize global objects.
        //
        opMode = FtcOpMode.getInstance();
        opMode.hardwareMap.logDevices();
        dashboard = HalDashboard.getInstance();
        tracer = FtcOpMode.getGlobalTracer();
        dashboard.setTextView(
                (TextView)((FtcRobotControllerActivity)opMode.hardwareMap.appContext).findViewById(R.id.textOpMode));
        battery = new FtcRobotBattery();
        androidTone = new FtcAndroidTone("AndroidTone");
        if (USE_SPEECH)
        {
            textToSpeech = FtcOpMode.getInstance().getTextToSpeech();
            textToSpeech.speak("Init starting", TextToSpeech.QUEUE_FLUSH, null);
        }
        //
        // Initialize sensors.
        //
        imu = new FtcBNO055Imu("imu2");
        gyro = imu.gyro;
        //
        // Initialize vision subsystems.
        //
        if (USE_VUFORIA)
        {
            int cameraViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                    "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
            vuforiaVision = new VuforiaVision(this, -1);//cameraViewId);
        }
        //
        // Initialize DriveBase.
        //
        leftFrontWheel = new FtcDcMotor("leftFrontWheel");
        rightFrontWheel = new FtcDcMotor("rightFrontWheel");
        leftRearWheel = new FtcDcMotor("leftRearWheel");
        rightRearWheel = new FtcDcMotor("rightRearWheel");

        leftFrontWheel.motor.setMode(RobotInfo.DRIVE_MOTOR_MODE);
        rightFrontWheel.motor.setMode(RobotInfo.DRIVE_MOTOR_MODE);
        leftRearWheel.motor.setMode(RobotInfo.DRIVE_MOTOR_MODE);
        rightRearWheel.motor.setMode(RobotInfo.DRIVE_MOTOR_MODE);

        leftFrontWheel.setInverted(false);
        leftRearWheel.setInverted(false);
        rightFrontWheel.setInverted(true);
        rightRearWheel.setInverted(true);

        leftFrontWheel.setBrakeModeEnabled(true);
        leftRearWheel.setBrakeModeEnabled(true);
        rightFrontWheel.setBrakeModeEnabled(true);
        rightRearWheel.setBrakeModeEnabled(true);

        driveBase = new TrcMecanumDriveBase(leftFrontWheel, leftRearWheel, rightFrontWheel, rightRearWheel, gyro);
        driveBase.setPositionScales(RobotInfo.ENCODER_X_INCHES_PER_COUNT, RobotInfo.ENCODER_Y_INCHES_PER_COUNT);
        //
        // Initialize PID drive.
        //
        encoderXPidCtrl = new TrcPidController(
                "encoderXPidCtrl",
                new TrcPidController.PidCoefficients(
                        RobotInfo.ENCODER_X_KP, RobotInfo.ENCODER_X_KI, RobotInfo.ENCODER_X_KD),
                RobotInfo.ENCODER_X_TOLERANCE, driveBase::getXPosition);
        encoderYPidCtrl = new TrcPidController(
                "encoderYPidCtrl",
                new TrcPidController.PidCoefficients(
                        RobotInfo.ENCODER_Y_KP, RobotInfo.ENCODER_Y_KI, RobotInfo.ENCODER_Y_KD),
                RobotInfo.ENCODER_Y_TOLERANCE, driveBase::getYPosition);
        gyroPidCtrl = new TrcPidController(
                "gyroPidCtrl",
                new TrcPidController.PidCoefficients(
                        RobotInfo.GYRO_KP, RobotInfo.GYRO_KI, RobotInfo.GYRO_KD),
                RobotInfo.GYRO_TOLERANCE, driveBase::getHeading);
        gyroPidCtrl.setAbsoluteSetPoint(true);
        gyroPidCtrl.setOutputRange(-RobotInfo.TURN_POWER_LIMIT, RobotInfo.TURN_POWER_LIMIT);

        pidDrive = new TrcPidDrive("pidDrive", driveBase, encoderXPidCtrl, encoderYPidCtrl, gyroPidCtrl);
        pidDrive.setStallTimeout(RobotInfo.PIDDRIVE_STALL_TIMEOUT);
        pidDrive.setBeep(androidTone);

        //
        // Initialize other subsystems.
        //

        //
        // Tell the driver initialization is complete.
        //
        if (textToSpeech != null)
        {
            textToSpeech.speak("Init complete!", TextToSpeech.QUEUE_ADD, null);
        }
    }   //Robot

    void startMode(TrcRobot.RunMode runMode)
    {
        //
        // Since the IMU gyro is giving us cardinal heading, we need to enable its cardinal to cartesian converter.
        //
        gyro.resetZIntegrator();
        gyro.setEnabled(true);

        if (vuforiaVision != null)
        {
            vuforiaVision.setEnabled(true);
        }

        //
        // Reset all X, Y and heading values.
        //
        driveBase.resetOdometry();
        targetHeading = 0.0;
    }   //startMode

    void stopMode(TrcRobot.RunMode runMode)
    {
        if (vuforiaVision != null)
        {
            vuforiaVision.setEnabled(false);
        }
        //
        // Disable the gyro integrator.
        //
        gyro.setEnabled(false);

        if (textToSpeech != null)
        {
            textToSpeech.stop();
            textToSpeech.shutdown();
        }
    }   //stopMode

    void traceStateInfo(double elapsedTime, String stateName, double xDistance, double yDistance, double heading)
    {
        tracer.traceInfo(
                moduleName,
                "[%5.3f] >>>>> %s: xPos=%6.2f/%6.2f,yPos=%6.2f/%6.2f,heading=%6.1f/%6.1f,volt=%5.2fV(%5.2fV)",
                elapsedTime, stateName,
                driveBase.getXPosition(), xDistance, driveBase.getYPosition(), yDistance,
                driveBase.getHeading(), heading,
                battery.getVoltage(), battery.getLowestVoltage());
    }   //traceStateInfo

    //
    // Implements FtcMenu.MenuButtons interface.
    //

    @Override
    public boolean isMenuUpButton()
    {
        return opMode.gamepad1.dpad_up;
    }   //isMenuUpButton

    @Override
    public boolean isMenuDownButton()
    {
        return opMode.gamepad1.dpad_down;
    }   //isMenuDownButton

    @Override
    public boolean isMenuEnterButton()
    {
        return opMode.gamepad1.a;
    }   //isMenuEnterButton

    @Override
    public boolean isMenuBackButton()
    {
        return opMode.gamepad1.dpad_left;
    }   //isMenuBackButton

}   //class Robot
