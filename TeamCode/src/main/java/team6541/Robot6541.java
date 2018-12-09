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

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import common.MineralScooper;
import common.PixyVision;
import common.Robot;
import common.RobotInfo;
import common.TeamMarkerDeployer;
import common.TensorFlowVision;
import common.VuforiaVision;
import ftclib.FtcDcMotor;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcRobot;
import trclib.TrcSimpleDriveBase;
import trclib.TrcUtil;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class Robot6541 extends Robot
{
    public static final boolean USE_VUFORIA = false;
    public static final boolean USE_PIXY = false;
    public static final boolean USE_TENSORFLOW = true;

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
        // Initialize vision subsystems.
        //
        if (USE_VUFORIA)
        {
            final int cameraViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                    "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
            final VuforiaLocalizer.CameraDirection CAMERA_DIR = BACK;
            /*
             * Create a transformation matrix describing where the phone is on the robot.
             *
             * The coordinate frame for the robot looks the same as the field.
             * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
             * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
             *
             * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
             * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
             * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
             *
             * If using the rear (High Res) camera:
             * We need to rotate the camera around it's long axis to bring the rear camera forward.
             * This requires a negative 90 degree rotation on the Y axis
             *
             * If using the Front (Low Res) camera
             * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
             * This requires a Positive 90 degree rotation on the Y axis
             *
             * Next, translate the camera lens to where it is on the robot.
             * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
             */
            final double ROBOT_LENGTH = 17.5;           //Robot length in inches
            final double ROBOT_WIDTH = 17.5;            //Robot width in inches
            final double PHONE_FRONT_OFFSET = 1.25;     //Phone offset from front of robot in inches
            final double PHONE_HEIGHT_OFFSET = 6.25;    //Phone offset from the floor in inches
            final double PHONE_LEFT_OFFSET = 4.75;      //Phone offset from the left side of the robot in inches
            final int CAMERA_FORWARD_DISPLACEMENT = (int)((ROBOT_LENGTH/2.0 - PHONE_FRONT_OFFSET)*TrcUtil.MM_PER_INCH);
            final int CAMERA_VERTICAL_DISPLACEMENT = (int)(PHONE_HEIGHT_OFFSET*TrcUtil.MM_PER_INCH);
            final int CAMERA_LEFT_DISPLACEMENT = (int)((ROBOT_WIDTH/2.0 - PHONE_LEFT_OFFSET)*TrcUtil.MM_PER_INCH);

            OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                            CAMERA_DIR == BACK? -90: 90, 0, 0));

            vuforiaVision = new VuforiaVision(this, /*cameraViewId*/-1, CAMERA_DIR, phoneLocationOnRobot);
        }

        if (USE_PIXY)
        {
            pixyVision = new PixyVision("pixy", this,
                    PixyVision.Orientation.NORMAL_LANDSCAPE, 80);
        }
        //
        // TensorFlow slows down our threads really badly, so don't enable it if we don't need it.
        //
        if (USE_TENSORFLOW && (runMode == TrcRobot.RunMode.AUTO_MODE || runMode == TrcRobot.RunMode.TEST_MODE))
        {
            int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
            final VuforiaLocalizer.CameraDirection CAMERA_DIR = BACK;
            tensorFlowVision = new TensorFlowVision(-1/*tfodMonitorViewId*/, CAMERA_DIR, globalTracer);
            tensorFlowVision.setEnabled(true);
            globalTracer.traceInfo("Robot3543", "Enabling TensorFlow.");
        }
        //
        // Initialize DriveBase.
        //
        leftFrontWheel = new FtcDcMotor("lfWheel");
        rightFrontWheel = new FtcDcMotor("rfWheel");

        leftRearWheel = new RearWheelMotor("lrWheel", leftFrontWheel);
        rightRearWheel = new RearWheelMotor("rrWheel", rightFrontWheel);

        leftFrontWheel.motor.setMode(RobotInfo6541.DRIVE_MOTOR_MODE);
        rightFrontWheel.motor.setMode(RobotInfo6541.DRIVE_MOTOR_MODE);
        leftRearWheel.motor.setMode(RobotInfo6541.DRIVE_MOTOR_MODE);
        rightRearWheel.motor.setMode(RobotInfo6541.DRIVE_MOTOR_MODE);

        leftFrontWheel.setOdometryEnabled(true);
        rightFrontWheel.setOdometryEnabled(true);
        leftRearWheel.setOdometryEnabled(true);
        rightRearWheel.setOdometryEnabled(true);

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
        rightFrontWheel.setInverted(true);
        rightRearWheel.setInverted(true);

        leftFrontWheel.setBrakeModeEnabled(true);
        leftRearWheel.setBrakeModeEnabled(true);
        rightFrontWheel.setBrakeModeEnabled(true);
        rightRearWheel.setBrakeModeEnabled(true);

        driveBase = new TrcSimpleDriveBase(leftFrontWheel, leftRearWheel, rightFrontWheel, rightRearWheel, gyro);
        driveBase.setPositionScales(RobotInfo6541.ENCODER_Y_INCHES_PER_COUNT);
        //
        // Initialize PID drive.
        //
        encoderYPidCtrl = new TrcPidController(
                "encoderYPidCtrl",
                new TrcPidController.PidCoefficients(
                        RobotInfo6541.ENCODER_Y_KP, RobotInfo6541.ENCODER_Y_KI, RobotInfo6541.ENCODER_Y_KD),
                RobotInfo6541.ENCODER_Y_TOLERANCE, driveBase::getYPosition);
        gyroPidCtrl = new TrcPidController(
                "gyroPidCtrl",
                new TrcPidController.PidCoefficients(
                        RobotInfo6541.GYRO_KP, RobotInfo6541.GYRO_KI, RobotInfo6541.GYRO_KD),
                RobotInfo6541.GYRO_TOLERANCE, this::getHeading);
        gyroPidCtrl.setAbsoluteSetPoint(true);
        gyroPidCtrl.setOutputRange(-RobotInfo6541.TURN_POWER_LIMIT, RobotInfo6541.TURN_POWER_LIMIT);

        pidDrive = new TrcPidDrive("pidDrive", driveBase, null, encoderYPidCtrl, gyroPidCtrl);
        pidDrive.setStallTimeout(RobotInfo6541.PIDDRIVE_STALL_TIMEOUT);
        pidDrive.setBeep(androidTone);
        pidDrive.setMsgTracer(globalTracer, true);
        //
        // Initialize other subsystems.
        //
        elevator = new Elevator6541();
        teamMarkerDeployer = new TeamMarkerDeployer(
                RobotInfo6541.DEPLOYER_OPEN_POSITION, RobotInfo6541.DEPLOYER_CLOSE_POSITION);
        mineralScooper = new MineralScooper(
                RobotInfo6541.MINERAL_SCOOPER_EXTEND_POSITION, RobotInfo6541.MINERAL_SCOOPER_RETRACT_POSITION);
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
