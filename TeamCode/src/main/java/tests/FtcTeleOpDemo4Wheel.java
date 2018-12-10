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

package tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftclib.FtcDcMotor;
import ftclib.FtcGamepad;
import ftclib.FtcOpMode;
import hallib.HalDashboard;
import trclib.TrcGameController;
import trclib.TrcMecanumDriveBase;

@TeleOp(name="Test: Demo 4-Wheel", group="Test")
//@Disabled
public class FtcTeleOpDemo4Wheel extends FtcOpMode
{
    private static final String moduleName = "FtcTeleOpDemo4Wheel";

    protected enum DriveMode
    {
        TANK_MODE,
        MECANUM_MODE,
    }   //enum DriveMode

    private static final boolean LEFT_WHEEL_INVERTED = false;
    private static final boolean RIGHT_WHEEL_INVERTED = true;
    private static final boolean BRAKE_MODE_ENABLED = true;

    private HalDashboard dashboard;
    private FtcGamepad driverGamepad;

    private FtcDcMotor lfWheel;
    private FtcDcMotor rfWheel;
    private FtcDcMotor lrWheel;
    private FtcDcMotor rrWheel;
    private TrcMecanumDriveBase driveBase;

    private DriveMode driveMode = DriveMode.MECANUM_MODE;
    private boolean invertedDrive = false;

    @Override
    public void initRobot()
    {
        dashboard = HalDashboard.getInstance();

        driverGamepad = new FtcGamepad("DriverGamepad", gamepad1, this::buttonEvent);
        driverGamepad.setYInverted(true);

        lfWheel = new FtcDcMotor("lfWheel");
        rfWheel = new FtcDcMotor("rfWheel");
        lrWheel = new FtcDcMotor("lrWheel");
        rrWheel = new FtcDcMotor("rrWheel");

        lfWheel.setInverted(LEFT_WHEEL_INVERTED);
        lrWheel.setInverted(LEFT_WHEEL_INVERTED);
        rfWheel.setInverted(RIGHT_WHEEL_INVERTED);
        rrWheel.setInverted(RIGHT_WHEEL_INVERTED);

        lfWheel.setBrakeModeEnabled(BRAKE_MODE_ENABLED);
        rfWheel.setBrakeModeEnabled(BRAKE_MODE_ENABLED);
        lrWheel.setBrakeModeEnabled(BRAKE_MODE_ENABLED);
        rrWheel.setBrakeModeEnabled(BRAKE_MODE_ENABLED);

        driveBase = new TrcMecanumDriveBase(lfWheel, lrWheel, rfWheel, rrWheel);
    }   //initRobot

    @Override
    public void runPeriodic(double elapsedTime)
    {
        //
        // DriveBase subsystem.
        //
        switch (driveMode)
        {
            case TANK_MODE:
                double leftPower = driverGamepad.getLeftStickY(true);
                double rightPower = driverGamepad.getRightStickY(true);
                driveBase.tankDrive(leftPower, rightPower, invertedDrive);
                dashboard.displayPrintf(1, "Tank:left=%.1f,right=%.1f,inv=%s",
                        leftPower, rightPower, Boolean.toString(invertedDrive));
                break;

            case MECANUM_MODE:
                double x = driverGamepad.getLeftStickX(true);
                double y = driverGamepad.getRightStickY(true);
                double rot = (driverGamepad.getRightTrigger(true) - driverGamepad.getLeftTrigger(true));
                driveBase.holonomicDrive(x, y, rot, invertedDrive);
                dashboard.displayPrintf(1, "Mecan:x=%.1f,y=%.1f,rot=%.1f,inv=%s",
                        x, y, rot, Boolean.toString(invertedDrive));
                break;
        }

        dashboard.displayPrintf(2, "DriveBase: x=%.2f,y=%.2f,heading=%.2f",
                driveBase.getXPosition(), driveBase.getYPosition(), driveBase.getHeading());
    }   //runPeriodic

    public void buttonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        dashboard.displayPrintf(
                7, "%s: %04x->%s", gamepad.toString(), button, pressed? "Pressed": "Released");

        if (gamepad == driverGamepad)
        {
            switch (button)
            {
                case FtcGamepad.GAMEPAD_A:
                    if (pressed)
                    {
                        driveMode = DriveMode.MECANUM_MODE;
                    }
                    break;

                case FtcGamepad.GAMEPAD_B:
                    if (pressed)
                    {
                        driveMode = DriveMode.TANK_MODE;
                    }
                    break;

                case FtcGamepad.GAMEPAD_X:
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    break;

                case FtcGamepad.GAMEPAD_LBUMPER:
                    break;

                case FtcGamepad.GAMEPAD_RBUMPER:
                    invertedDrive = pressed;
                    break;
            }
        }
    }   //buttonEvent

}   //class FtcTeleOpDemo4Wheel
