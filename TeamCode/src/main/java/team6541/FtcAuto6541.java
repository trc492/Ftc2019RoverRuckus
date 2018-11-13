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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import common.AutoCommon;
import common.CmdPidDrive;
import common.CmdTimedDrive;
import trclib.TrcRobot;

@Autonomous(name="Autonomous6541", group="Auto")
public class FtcAuto6541 extends AutoCommon
{
    private static final String moduleName = "FtcAuto6541";
    private Robot6541 robot;

    //
    // Implements FtcOpMode abstract method.
    //

    @Override
    public void initRobot()
    {
        //
        // Initializing robot objects.
        //
        robot = new Robot6541(TrcRobot.RunMode.AUTO_MODE);
        super.setRobot(robot);
        //
        // Choice menus.
        //
        doMenus();

        if (USE_TRACELOG)
        {
            String filePrefix = String.format("%s%02d", matchType, matchNumber);
            robot.tracer.openTraceLog("/sdcard/FIRST/tracelog", filePrefix);
        }

        //
        // Strategies.
        //
        switch (strategy)
        {
            case CRATER_AUTO:
                autoCommand = new CmdAutoCrater6541(robot, alliance, delay, startHung, doMineral, doTeamMarker,
                        doTeammateMineral);
                break;

            case DEPOT_AUTO:
                autoCommand = new CmdAutoDepot6541(robot, alliance, delay, startHung, doMineral, doTeamMarker);
                break;

            case DISTANCE_DRIVE:
                autoCommand = new CmdPidDrive(
                        robot, robot.pidDrive, delay, 0.0, driveDistance*12.0, 0.0);
                break;

            case TIMED_DRIVE:
                autoCommand = new CmdTimedDrive(robot, delay, driveTime, 0.0, drivePower, 0.0);
                break;

            case DO_NOTHING:
            default:
                autoCommand = null;
                break;
        }
    }   //initRobot

}   //class FtcAuto6541
