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

import android.speech.tts.TextToSpeech;
import android.widget.TextView;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

import ftclib.FtcAndroidTone;
import ftclib.FtcBNO055Imu;
import ftclib.FtcDcMotor;
import ftclib.FtcMenu;
import ftclib.FtcOpMode;
import ftclib.FtcRobotBattery;
import hallib.HalDashboard;
import team3543.R;
import trclib.TrcDbgTrace;
import trclib.TrcGyro;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcRobot;
import trclib.TrcSimpleDriveBase;

public class Robot implements FtcMenu.MenuButtons
{
    public static final boolean USE_SPEECH = true;
    public static final boolean USE_VUFORIA = false;
    public static final boolean USE_PIXY = true;
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
    // Global objects.
    //
    public String moduleName;
    public FtcOpMode opMode;
    public HalDashboard dashboard;
    public TrcDbgTrace tracer;
    public FtcRobotBattery battery = null;
    public FtcAndroidTone androidTone;
    public TextToSpeech textToSpeech = null;
    //
    // Vision subsystems.
    //
    public VuforiaVision vuforiaVision = null;
    public PixyVision pixyVision = null;
    //
    // Sensors.
    //
    public FtcBNO055Imu imu = null;
    public TrcGyro gyro = null;
    public double targetHeading = 0.0;
    public double headingOffset = 0.0;
    //
    // DriveBase subsystem.
    //
    public FtcDcMotor leftFrontWheel = null;
    public FtcDcMotor rightFrontWheel = null;
    public FtcDcMotor leftRearWheel = null;
    public FtcDcMotor rightRearWheel = null;

    public TrcPidController encoderXPidCtrl = null;
    public TrcPidController encoderYPidCtrl = null;
    public TrcPidController gyroPidCtrl = null;
    public TrcSimpleDriveBase driveBase = null;
    public TrcPidDrive pidDrive = null;

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
            speak("Init starting");
        }
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
            pixyVision = new PixyVision("pixy", this, PIXY_ORIENTATION, PIXY_BRIGHTNESS);
        }
        //
        // Initialize sensors.
        //
        imu = new FtcBNO055Imu("imu");
        gyro = imu.gyro;
    }   //Robot

    public void alignHeadingWithVuforia(double defOrientation)
    {
        final String funcName = "alignHeadingWithVuforia";

        OpenGLMatrix robotLocation = vuforiaVision == null? null: vuforiaVision.getRobotLocation();
        double robotOrientation = defOrientation;

        if (robotLocation != null)
        {
            robotOrientation = vuforiaVision.getRobotTranslation(robotLocation).get(3);
            speak(String.format("Robot angle is %.1f degrees", robotOrientation));
        }

        tracer.traceInfo(funcName, "orientation=%.3f", robotOrientation);
        targetHeading = robotOrientation;
        headingOffset = robotOrientation - driveBase.getHeading();
    }   //alignHeading

    public double getHeading()
    {
        return driveBase.getHeading() + headingOffset;
    }   //getHeading

    public void speak(String sentence)
    {
        if (textToSpeech != null)
        {
            textToSpeech.speak(sentence, TextToSpeech.QUEUE_FLUSH, null);
        }
    }   //speak

    public void startMode(TrcRobot.RunMode runMode)
    {
        //
        // Since the IMU gyro is giving us cardinal heading, we need to enable its cardinal to cartesian converter.
        //
        gyro.resetZIntegrator();
        gyro.setEnabled(true);
        targetHeading = 0.0;
    }   //startMode

    public void stopMode(TrcRobot.RunMode runMode)
    {
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

    public void traceStateInfo(double elapsedTime, String stateName, double xDistance, double yDistance, double heading)
    {
        tracer.traceInfo(
                moduleName,
                "[%5.3f] >>>>> %s: xPos=%6.2f/%6.2f,yPos=%6.2f/%6.2f,heading=%6.1f/%6.1f,volt=%5.2fV(%5.2fV)",
                elapsedTime, stateName,
                driveBase.getXPosition(), xDistance, driveBase.getYPosition(), yDistance,
                getHeading(), heading, battery.getVoltage(), battery.getLowestVoltage());
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
