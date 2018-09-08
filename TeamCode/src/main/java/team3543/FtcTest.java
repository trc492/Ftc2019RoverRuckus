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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import ftclib.FtcChoiceMenu;
import ftclib.FtcGamepad;
import ftclib.FtcMRRangeSensor;
import ftclib.FtcMenu;
import ftclib.FtcValueMenu;
import trclib.TrcEvent;
import trclib.TrcGameController;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

@TeleOp(name="Test", group="3543Test")
public class FtcTest extends FtcTeleOp implements TrcGameController.ButtonHandler
{
    private static final String moduleName = "FtcTest";

    private enum Test
    {
        SENSORS_TEST,
        MOTORS_TEST,
        X_TIMED_DRIVE,
        Y_TIMED_DRIVE,
        X_DISTANCE_DRIVE,
        Y_DISTANCE_DRIVE,
        GYRO_TURN,
        VISION_TEST,
        VISION_DRIVE,
        LEFT_RANGE_DRIVE,
        RIGHT_RANGE_DRIVE,
        LEFT_SONAR_DRIVE,
        FRONT_SONAR_DRIVE,
        RIGHT_SONAR_DRIVE
    }   //enum Test

    private enum State
    {
        START,
        STOP,
        DONE
    }   //enum State

    //
    // State machine.
    //
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;
    //
    // Menu choices.
    //
    private Test test = Test.SENSORS_TEST;
    private double driveTime = 0.0;
    private double driveDistance = 0.0;
    private double turnDegrees = 0.0;
    private double rangeDistance = 0.0;
    private double sonarDistance = 0.0;

    private CmdTimedDrive timedDriveCommand = null;
    private CmdPidDrive pidDriveCommand = null;
    private CmdVisionDrive visionDriveCommand = null;
    private CmdPidDrive rangeDriveCommand = null;
    private CmdPidDrive sonarDriveCommand = null;

    private int motorIndex = 0;

    //
    // Implements FtcOpMode interface.
    //

    @Override
    public void initRobot()
    {
        //
        // FtcTest inherits from FtcTeleOp so it can do everything that FtcTeleOp can do and more.
        //
        super.initRobot();
        //
        // Initialize additional objects.
        //
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        //
        // Test menus.
        //
        doMenus();

        switch (test)
        {
            case X_TIMED_DRIVE:
                timedDriveCommand = new CmdTimedDrive(
                        robot, 0.0, driveTime, 1.0, 0.0, 0.0);
                break;

            case Y_TIMED_DRIVE:
                timedDriveCommand = new CmdTimedDrive(
                        robot, 0.0, driveTime, 0.0, 0.2, 0.0);
                break;

            case X_DISTANCE_DRIVE:
                pidDriveCommand = new CmdPidDrive(
                        robot, robot.pidDrive, 0.0, driveDistance*12.0, 0.0, 0.0);
                break;

            case Y_DISTANCE_DRIVE:
                pidDriveCommand = new CmdPidDrive(
                        robot, robot.pidDrive, 0.0, 0.0, driveDistance*12.0, 0.0);
                break;

            case GYRO_TURN:
                pidDriveCommand = new CmdPidDrive(
                        robot, robot.pidDrive, 0.0, 0.0, 0.0, turnDegrees);
                break;

            case VISION_DRIVE:
                visionDriveCommand = new CmdVisionDrive(robot);
                break;

            case LEFT_RANGE_DRIVE:
            case RIGHT_RANGE_DRIVE:
                if (Robot.USE_MRRANGE_SENSOR)
                {
                    robot.useRightSensorForX = test == Test.RIGHT_RANGE_DRIVE;
                    robot.rangeXPidCtrl.setInverted(robot.useRightSensorForX);
                    rangeDriveCommand = new CmdPidDrive(
                            robot, robot.rangeXPidDrive, 0.0, rangeDistance, 0.0, 0.0);
                }
                break;

            case LEFT_SONAR_DRIVE:
            case RIGHT_SONAR_DRIVE:
                if (Robot.USE_MAXBOTIX_SONAR_SENSOR)
                {
                    robot.useRightSensorForX = test == Test.RIGHT_SONAR_DRIVE;
                    robot.sonarXPidCtrl.setInverted(robot.useRightSensorForX);
                    sonarDriveCommand = new CmdPidDrive(
                            robot, robot.sonarXPidDrive, 0.0, sonarDistance, 0.0, 0.0);
                }
                break;

            case FRONT_SONAR_DRIVE:
                if (Robot.USE_MAXBOTIX_SONAR_SENSOR)
                {
                    sonarDriveCommand = new CmdPidDrive(
                            robot, robot.sonarYPidDrive, 0.0, 0.0, sonarDistance, 0.0);
                }
                break;
        }

        sm.start(State.START);
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode()
    {
        super.startMode();

        if (test == Test.SENSORS_TEST)
        {
            if (robot.jewelColorTrigger != null)
            {
                robot.jewelColorTrigger.setEnabled(true);
            }

            if (robot.cryptoColorTrigger != null)
            {
                robot.redCryptoBarCount = 0;
                robot.blueCryptoBarCount = 0;
                robot.cryptoColorTrigger.setEnabled(true);
            }
        }

        if (robot.sonarArray != null)
        {
            robot.sonarArray.startRanging(true);
        }
    }   //startMode

    @Override
    public void stopMode()
    {
        super.stop();

        if (robot.sonarArray != null)
        {
            robot.sonarArray.stopRanging();
        }

        if (test == Test.SENSORS_TEST)
        {
            if (robot.jewelColorTrigger != null)
            {
                robot.jewelColorTrigger.setEnabled(false);
            }

            if (robot.cryptoColorTrigger != null)
            {
                robot.cryptoColorTrigger.setEnabled(false);
            }
        }

        printPerformanceMetrics(robot.tracer);
    }   //stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        //
        // Must override TeleOp so it doesn't fight with us.
        //
        switch (test)
        {
            case SENSORS_TEST:
            case VISION_TEST:
                //
                // Allow TeleOp to run so we can control the robot in sensors test mode.
                //
                super.runPeriodic(elapsedTime);
                doSensorsTest();
                doVisionTest();
                break;

            case MOTORS_TEST:
                doMotorsTest();
                break;
        }
    }   //runPeriodic

    @Override
    public void runContinuous(double elapsedTime)
    {
        State state = sm.getState();
        dashboard.displayPrintf(8, "%s: %s", test.toString(), state != null? state.toString(): "STOPPED!");

        switch (test)
        {
            case X_TIMED_DRIVE:
            case Y_TIMED_DRIVE:
                double lfEnc = robot.leftFrontWheel.getPosition();
                double rfEnc = robot.rightFrontWheel.getPosition();
                double lrEnc = robot.leftRearWheel.getPosition();
                double rrEnc = robot.rightRearWheel.getPosition();
                dashboard.displayPrintf(9, "Timed Drive: %.0f sec", time);
                dashboard.displayPrintf(10, "Enc:lf=%.0f,rf=%.0f", lfEnc, rfEnc);
                dashboard.displayPrintf(11, "Enc:lr=%.0f,rr=%.0f", lrEnc, rrEnc);
                dashboard.displayPrintf(12, "average=%f", (lfEnc + rfEnc + lrEnc + rrEnc)/4.0);
                dashboard.displayPrintf(13, "xPos=%.1f,yPos=%.1f,heading=%.1f",
                                        robot.driveBase.getXPosition(),
                                        robot.driveBase.getYPosition(),
                                        robot.driveBase.getHeading());
                timedDriveCommand.cmdPeriodic(elapsedTime);
                break;

            case X_DISTANCE_DRIVE:
            case Y_DISTANCE_DRIVE:
            case GYRO_TURN:
                dashboard.displayPrintf(9, "xPos=%.1f,yPos=%.1f,heading=%.1f",
                                        robot.getInput(robot.encoderXPidCtrl),
                                        robot.getInput(robot.encoderYPidCtrl),
                                        robot.getInput(robot.gyroPidCtrl));
                robot.encoderXPidCtrl.displayPidInfo(10);
                robot.encoderYPidCtrl.displayPidInfo(12);
                robot.gyroPidCtrl.displayPidInfo(14);

                pidDriveCommand.cmdPeriodic(elapsedTime);
                break;

            case VISION_DRIVE:
                dashboard.displayPrintf(9, "visionTarget=%.1f,heading=%.1f",
                        robot.getInput(robot.visionPidCtrl),
                        robot.getInput(robot.gyroPidCtrl));
                robot.visionPidCtrl.displayPidInfo(10);
                robot.gyroPidCtrl.displayPidInfo(12);

                visionDriveCommand.cmdPeriodic(elapsedTime);
                break;

            case LEFT_RANGE_DRIVE:
            case RIGHT_RANGE_DRIVE:
                if (rangeDriveCommand != null)
                {
                    dashboard.displayPrintf(9, "rangeXDist=%.1f,heading=%.1f",
                            robot.getInput(robot.rangeXPidCtrl), robot.getInput(robot.gyroPidCtrl));
                    robot.rangeXPidCtrl.displayPidInfo(10);
                    robot.gyroPidCtrl.displayPidInfo(12);

                    rangeDriveCommand.cmdPeriodic(elapsedTime);
                }
                else
                {
                    dashboard.displayPrintf(9, "Range sensor is disabled.");
                }
                break;

            case LEFT_SONAR_DRIVE:
            case FRONT_SONAR_DRIVE:
            case RIGHT_SONAR_DRIVE:
                if (sonarDriveCommand != null)
                {
                    dashboard.displayPrintf(9, "sonarXDist=%.1f,sonarYDist=%.1f,heading=%.1f",
                            robot.getInput(robot.sonarXPidCtrl),
                            robot.getInput(robot.sonarYPidCtrl),
                            robot.getInput(robot.gyroPidCtrl));
                    robot.sonarXPidCtrl.displayPidInfo(10);
                    robot.sonarYPidCtrl.displayPidInfo(12);
                    robot.gyroPidCtrl.displayPidInfo(14);

                    sonarDriveCommand.cmdPeriodic(elapsedTime);
                }
                else
                {
                    dashboard.displayPrintf(9, "Sonar sensors are disabled.");
                }
                break;
        }
    }   //runContinuous

    private void doMenus()
    {
        //
        // Create menus.
        //
        FtcChoiceMenu<Test> testMenu = new FtcChoiceMenu<>("Tests:", null, robot);
        FtcValueMenu driveTimeMenu = new FtcValueMenu(
                "Drive time:", testMenu, robot, 1.0, 10.0, 1.0, 4.0,
                " %.0f sec");
        FtcValueMenu driveDistanceMenu = new FtcValueMenu(
                "Drive distance:", testMenu, robot, -10.0, 10.0, 0.5, 4.0,
                " %.1f ft");
        FtcValueMenu turnDegreesMenu = new FtcValueMenu(
                "Turn degrees:", testMenu, robot, -360.0, 360.0, 5.0, 45.0,
                " %.0f deg");
        FtcValueMenu rangeDistanceMenu = new FtcValueMenu(
                "Range distance:", testMenu, robot, 1.0, 50.0, 1.0, 12.0,
                " %.0f in");
        FtcValueMenu sonarDistanceMenu = new FtcValueMenu(
                "Sonar distance:", testMenu, robot, 6.0, 50.0, 1.0, 12.0,
                " %.0f in");

        //
        // Populate menus.
        //
        testMenu.addChoice("Sensors test", Test.SENSORS_TEST, true);
        testMenu.addChoice("Motors test", Test.MOTORS_TEST, false);
        testMenu.addChoice("X Timed drive", Test.X_TIMED_DRIVE, false, driveTimeMenu);
        testMenu.addChoice("Y Timed drive", Test.Y_TIMED_DRIVE, false, driveTimeMenu);
        testMenu.addChoice("X Distance drive", Test.X_DISTANCE_DRIVE, false, driveDistanceMenu);
        testMenu.addChoice("Y Distance drive", Test.Y_DISTANCE_DRIVE, false, driveDistanceMenu);
        testMenu.addChoice("Degrees turn", Test.GYRO_TURN, false, turnDegreesMenu);
        testMenu.addChoice("Vision test", Test.VISION_TEST, false);
        testMenu.addChoice("Vision drive", Test.VISION_DRIVE, false);
        if (robot.USE_RANGE_DRIVE)
        {
            testMenu.addChoice("Left Range drive", Test.LEFT_RANGE_DRIVE, false, rangeDistanceMenu);
            testMenu.addChoice("Right Range drive", Test.RIGHT_RANGE_DRIVE, false, rangeDistanceMenu);
        }
        if (robot.USE_SONAR_DRIVE)
        {
            testMenu.addChoice("Left Sonar drive", Test.LEFT_SONAR_DRIVE, false, sonarDistanceMenu);
            testMenu.addChoice("Front Sonar drive", Test.FRONT_SONAR_DRIVE, false, sonarDistanceMenu);
            testMenu.addChoice("Right Sonar drive", Test.RIGHT_SONAR_DRIVE, false, sonarDistanceMenu);
        }
        //
        // Traverse menus.
        //
        FtcMenu.walkMenuTree(testMenu, this);
        //
        // Fetch choices.
        //
        test = testMenu.getCurrentChoiceObject();
        driveTime = driveTimeMenu.getCurrentValue();
        driveDistance = driveDistanceMenu.getCurrentValue();
        turnDegrees = turnDegreesMenu.getCurrentValue();
        rangeDistance = rangeDistanceMenu.getCurrentValue();
        sonarDistance = sonarDistanceMenu.getCurrentValue();
        //
        // Show choices.
        //
        dashboard.displayPrintf(0, "Test: %s", testMenu.getCurrentChoiceText());
    }   //doMenus

    /**
     * This method reads all sensors and prints out their values. This is a very useful diagnostic tool to check
     * if all sensors are working properly. For encoders, since test sensor mode is also teleop mode, you can
     * operate the gamepads to turn the motors and check the corresponding encoder counts.
     */
    private void doSensorsTest()
    {
        final int LABEL_WIDTH = 100;
        //
        // Read all sensors and display on the dashboard.
        // Drive the robot around to sample different locations of the field.
        //
        dashboard.displayPrintf(3, LABEL_WIDTH, "Enc: ", "lf=%.0f,rf=%.0f,lr=%.0f,rr=%.0f",
                                robot.leftFrontWheel.getPosition(), robot.rightFrontWheel.getPosition(),
                                robot.leftRearWheel.getPosition(), robot.rightRearWheel.getPosition());

        if (robot.gyro != null)
        {
            dashboard.displayPrintf(4, LABEL_WIDTH, "Gyro: ", "Rate=%.3f,Heading=%.1f",
                    robot.gyro.getZRotationRate().value, robot.gyro.getZHeading().value);
        }

        if (robot.sonarArray != null)
        {
            dashboard.displayPrintf(5, LABEL_WIDTH, "Sonar: ", "L=%.3f,F=%.3f,R=%.3f",
                    robot.sonarArray.getDistance(robot.LEFT_SONAR_INDEX).value,
                    robot.sonarArray.getDistance(robot.FRONT_SONAR_INDEX).value,
                    robot.sonarArray.getDistance(robot.RIGHT_SONAR_INDEX).value);
        }
        else if (robot.USE_MRRANGE_SENSOR)
        {
            dashboard.displayPrintf(5, LABEL_WIDTH, "MRRange: ", "L=%.1f,R=%.1f",
                    robot.getRangeDistance(robot.leftRangeSensor), robot.getRangeDistance(robot.rightRangeSensor));
        }

        dashboard.displayPrintf(
                6, LABEL_WIDTH, "Color: ", "Jewel=%s[%.0f/%.2f/%.2f]",
                robot.getObjectColor(robot.jewelColorSensor), robot.getObjectHsvHue(robot.jewelColorSensor),
                robot.getObjectHsvSaturation(robot.jewelColorSensor), robot.getObjectHsvValue(robot.jewelColorSensor));

        dashboard.displayPrintf(9, LABEL_WIDTH, "Elevator: ", "Pwr=%.2f,Pos=%.3f,low=%s",
                robot.glyphElevator.getPower(),
                robot.glyphElevator.getPosition(),
                robot.glyphElevator.elevatorLowerLimitSwitch.isActive());
        robot.glyphElevator.elevatorPidCtrl.displayPidInfo(10);

        dashboard.displayPrintf(12, LABEL_WIDTH, "RelicArm: ",
                "Elbow:Pwr=%.2f,Pos=%.1f,low=%s,high=%s",
                robot.relicArm.getElbowPower(),
                robot.relicArm.elbow.getPosition(),
                robot.relicArm.elbowLowerLimitSwitch.isActive(),
                robot.relicArm.elbowUpperLimitSwitch.isActive());
        robot.relicArm.elbowPidCtrl.displayPidInfo(13);

        dashboard.displayPrintf(15, LABEL_WIDTH, "Extender: ",
                "pwr=%.2f,low=%s,high=%s",
                robot.relicArm.extender.getPower(),
                robot.relicArm.extenderLowerLimitSwitch.isActive(),
                robot.relicArm.extenderUpperLimitSwitch.isActive());
    }   //doSensorsTest

    private void doVisionTest()
    {
        if (robot.vuforiaVision != null)
        {
            robot.vuforiaVision.getVuMarkPosition();
            robot.vuforiaVision.getVuMarkOrientation();
            RelicRecoveryVuMark vuMark = robot.vuforiaVision.getVuMark();
            if (vuMark != robot.prevVuMark)
            {
                String sentence = null;
                if (vuMark != RelicRecoveryVuMark.UNKNOWN)
                {
                    sentence = String.format("%s is %s.", vuMark.toString(), "in view");
                }
                else if (robot.prevVuMark != null)
                {
                    sentence = String.format("%s is %s.", robot.prevVuMark.toString(), "out of view");
                }

                if (sentence != null)
                {
                    robot.dashboard.displayPrintf(11, sentence);
                    if (robot.textToSpeech != null)
                    {
                        robot.textToSpeech.speak(sentence, TextToSpeech.QUEUE_FLUSH, null);
                    }
                }
            }
            robot.prevVuMark = vuMark;
        }
    }   //doVisionTest

    /**
     * This method runs each of the four wheels in sequence for a fixed number of seconds. It is for diagnosing
     * problems with the drive train. At the end of the run, you should check the amount of encoder counts each
     * wheel has accumulated. They should be about the same. If not, you need to check the problem wheel for
     * friction or chain tension etc. You can also use this test to check if a motor needs to be "inverted"
     * (i.e. turning in the wrong direction).
     */
    private void doMotorsTest()
    {
        double lfEnc = robot.leftFrontWheel.getPosition();
        double rfEnc = robot.rightFrontWheel.getPosition();
        double lrEnc = robot.leftRearWheel.getPosition();
        double rrEnc = robot.rightRearWheel.getPosition();

        dashboard.displayPrintf(9, "Motors Test: index=%d", motorIndex);
        dashboard.displayPrintf(10, "Enc: lf=%.0f, rf=%.0f", lfEnc, rfEnc);
        dashboard.displayPrintf(11, "Enc: lr=%.0f, rr=%.0f", lrEnc, rrEnc);

        if (sm.isReady())
        {
            State state = sm.getState();
            switch (state)
            {
                case START:
                    //
                    // Spin a wheel for 5 seconds.
                    //
                    switch (motorIndex)
                    {
                        case 0:
                            //
                            // Run the left front wheel.
                            //
                            robot.leftFrontWheel.setPower(0.5);
                            robot.rightFrontWheel.setPower(0.0);
                            robot.leftRearWheel.setPower(0.0);
                            robot.rightRearWheel.setPower(0.0);
                            break;

                        case 1:
                            //
                            // Run the right front wheel.
                            //
                            robot.leftFrontWheel.setPower(0.0);
                            robot.rightFrontWheel.setPower(0.5);
                            robot.leftRearWheel.setPower(0.0);
                            robot.rightRearWheel.setPower(0.0);
                            break;

                        case 2:
                            //
                            // Run the left rear wheel.
                            //
                            robot.leftFrontWheel.setPower(0.0);
                            robot.rightFrontWheel.setPower(0.0);
                            robot.leftRearWheel.setPower(0.5);
                            robot.rightRearWheel.setPower(0.0);
                            break;

                        case 3:
                            //
                            // Run the right rear wheel.
                            //
                            robot.leftFrontWheel.setPower(0.0);
                            robot.rightFrontWheel.setPower(0.0);
                            robot.leftRearWheel.setPower(0.0);
                            robot.rightRearWheel.setPower(0.5);
                            break;
                    }
                    motorIndex = motorIndex + 1;
                    timer.set(5.0, event);
                    sm.waitForSingleEvent(event, motorIndex < 4? State.START: State.STOP);
                    break;

                case STOP:
                    //
                    // We are done, stop all wheels.
                    //
                    robot.leftFrontWheel.setPower(0.0);
                    robot.rightFrontWheel.setPower(0.0);
                    robot.leftRearWheel.setPower(0.0);
                    robot.rightRearWheel.setPower(0.0);
                    sm.setState(State.DONE);
                    break;

                case DONE:
                default:
                    if (robot.textToSpeech != null)
                    {
                        double[] encCounts = {lfEnc, rfEnc, lrEnc, rrEnc};
                        double avgEnc = (lfEnc + rfEnc + lrEnc + rrEnc) / 4.0;
                        double minEnc = encCounts[0];
                        double maxEnc = encCounts[0];

                        for (int i = 1; i < encCounts.length; i++)
                        {
                            if (encCounts[i] < minEnc)
                                minEnc = encCounts[i];
                            else if (encCounts[i] > maxEnc)
                                maxEnc = encCounts[i];
                        }

                        if ((avgEnc - lfEnc) / avgEnc > 0.5)
                        {
                            robot.textToSpeech.speak(
                                    "left front wheel is stuck.", TextToSpeech.QUEUE_ADD, null);
                        }

                        if ((avgEnc - rfEnc) / avgEnc > 0.5)
                        {
                            robot.textToSpeech.speak(
                                    "right front wheel is stuck.", TextToSpeech.QUEUE_ADD, null);
                        }

                        if ((avgEnc - lrEnc) / avgEnc > 0.5)
                        {
                            robot.textToSpeech.speak(
                                    "left rear wheel is stuck.", TextToSpeech.QUEUE_ADD, null);
                        }

                        if ((avgEnc - rrEnc) / avgEnc > 0.5)
                        {
                            robot.textToSpeech.speak(
                                    "right rear wheel is stuck.", TextToSpeech.QUEUE_ADD, null);
                        }
                    }
                    sm.stop();
                    break;
            }
        }
    }   //doMotorsTest

    //
    // Overrides TrcGameController.ButtonHandler in FtcTeleOp.
    //

    @Override
    public void buttonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        boolean processed = false;
        //
        // In addition to or instead of the gamepad controls handled by FtcTeleOp, we can add to or override the
        // FtcTeleOp gamepad actions.
        //
        dashboard.displayPrintf(
                7, "%s: %04x->%s", gamepad.toString(), button, pressed? "Pressed": "Released");
        if (gamepad == driverGamepad)
        {
            switch (button)
            {
                case FtcGamepad.GAMEPAD_DPAD_UP:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_DOWN:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_LEFT:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                    break;
            }
        }
        else if (gamepad == operatorGamepad)
        {
            switch (button)
            {
                case FtcGamepad.GAMEPAD_DPAD_UP:
                    if (pressed)
                    {
                        robot.jewelArm.setExtended(false);
                    }
                    processed = true;
                    break;

                case FtcGamepad.GAMEPAD_DPAD_DOWN:
                    if (pressed)
                    {
                        robot.jewelArm.setExtended(true);
                    }
                    processed = true;
                    break;

                case FtcGamepad.GAMEPAD_DPAD_LEFT:
                    if (pressed)
                    {
                        robot.jewelArm.setSweepPosition(RobotInfo.JEWEL_ARM_BACKWARD);
                    }
                    else
                    {
                        robot.jewelArm.setSweepPosition(RobotInfo.JEWEL_ARM_NEUTRAL);
                    }
                    processed = true;
                    break;

                case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                    if (pressed)
                    {
                        robot.jewelArm.setSweepPosition(RobotInfo.JEWEL_ARM_FORWARD);
                    }
                    else
                    {
                        robot.jewelArm.setSweepPosition(RobotInfo.JEWEL_ARM_NEUTRAL);
                    }
                    processed = true;
                    break;
            }
        }
        //
        // If the control was not processed by this method, pass it back to FtcTeleOp.
        //
        if (!processed)
        {
            super.buttonEvent(gamepad, button, pressed);
        }
    }   //buttonEvent

}   //class FtcTest
