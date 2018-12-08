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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import common.TeleOpCommon;
import ftclib.FtcGamepad;
import trclib.TrcGameController;
import trclib.TrcRobot;

@TeleOp(name="TeleOp6541", group="FtcTeleOp")
public class FtcTeleOp6541 extends TeleOpCommon implements TrcGameController.ButtonHandler
{
    protected static final String moduleName = "FtcTeleOp6541";
    protected Robot6541 robot;

    //
    // Implements FtcOpMode abstract method.
    //

    @Override
    public void initRobot()
    {
        //
        // Initializing robot objects.
        // FtcTeleOp is also extended by FtcTest so we cannot assume runMode is TELEOP.
        //
        robot = new Robot6541(TrcRobot.getRunMode());
        driveMode = DriveMode.TANK_MODE;
        super.setRobot(robot);
        super.initRobot();
    }   //initRobot

    @Override
    public void runPeriodic(double elapsedTime)
    {
        super.runPeriodic(elapsedTime);

        double elevatorPower = operatorGamepad.getRightStickY(true);
        robot.elevator.setPower(elevatorPower);

        dashboard.displayPrintf(3, "Elevator: power=%.1f, pos=%.1f (%s,%s)",
                elevatorPower, robot.elevator.getPosition(),
                robot.elevator.isLowerLimitSwitchActive(), robot.elevator.isUpperLimitSwitchActive());
    }   //runPeriodic

    //
    // Implements TrcGameController.ButtonHandler interface.
    //

    @Override
    public void buttonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        dashboard.displayPrintf(
                7, "%s: %04x->%s", gamepad.toString(), button, pressed? "Pressed": "Released");
        if (gamepad == driverGamepad)
        {
            switch (button)
            {
                case FtcGamepad.GAMEPAD_A:
                    break;

                case FtcGamepad.GAMEPAD_B:
                    break;

                case FtcGamepad.GAMEPAD_X:
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    break;

                case FtcGamepad.GAMEPAD_LBUMPER:
                    drivePowerScale = pressed? 0.5: 1.0;
                    break;

                case FtcGamepad.GAMEPAD_RBUMPER:
                    invertedDrive = pressed;
                    break;
            }
        }
        else if (gamepad == operatorGamepad)
        {
            switch (button)
            {
                case FtcGamepad.GAMEPAD_A:
                    //
                    // extend the elevator to hanging height.
                    //
                    if (pressed)
                    {
                        robot.elevator.setPosition(RobotInfo6541.ELEVATOR_HANGING_HEIGHT);
                    }
                    break;

                case FtcGamepad.GAMEPAD_B:
                    break;

                case FtcGamepad.GAMEPAD_X:
                    if (pressed)
                    {
                        robot.teamMarkerDeployer.close();
                    }
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    if (pressed)
                    {
                        robot.teamMarkerDeployer.open();
                    }
                    break;

                case FtcGamepad.GAMEPAD_LBUMPER:
                    break;

                case FtcGamepad.GAMEPAD_RBUMPER:
                    robot.elevator.setManualOverride(pressed);
                    break;

                case FtcGamepad.GAMEPAD_DPAD_UP:
                    if (pressed)
                    {
                        robot.mineralScooper.extend();
                    }
                    break;

                case FtcGamepad.GAMEPAD_DPAD_DOWN:
                    if (pressed)
                    {
                        robot.mineralScooper.retract();
                    }
                    break;

                case FtcGamepad.GAMEPAD_DPAD_LEFT:
                    if (pressed)
                    {
                        robot.elevator.openHook();
                    }
                    break;

                case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                    if (pressed)
                    {
                        robot.elevator.closeHook();
                    }
                    break;

                case FtcGamepad.GAMEPAD_BACK:
                    if (pressed)
                    {
                        robot.elevator.zeroCalibrate();
                    }
                    break;

                case FtcGamepad.GAMEPAD_START:
                    if (robot.pixyVision != null && pressed)
                    {
                        robot.pixyVision.toggleCamera();
                    }
                    break;
            }
        }
    }   //buttonEvent

}   //class FtcTeleOp6541
