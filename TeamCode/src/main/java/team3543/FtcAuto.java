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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Date;

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcOpMode;
import ftclib.FtcValueMenu;
import trclib.TrcRobot;

@Autonomous(name="Autonomous", group="3543Auto")
public class FtcAuto extends FtcOpMode
{
    private static final boolean USE_TRACELOG = true;

    enum MatchType
    {
        PRACTICE,
        QUALIFICATION,
        SEMI_FINAL,
        FINAL
    }   //enum MatchType

    enum Alliance
    {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }   //enum Alliance

    enum StartPos
    {
        NEAR,
        FAR
    }

    enum DoJewel
    {
        YES,
        NO
    }

    enum DoCrypto
    {
        YES,
        NO
    }

    private enum Strategy
    {
        FULL_AUTO,
        DISTANCE_DRIVE,
        TIMED_DRIVE,
        DO_NOTHING
    }   //enum Strategy

    private static final String moduleName = "FtcAuto";

    private Robot robot;
    private TrcRobot.RobotCommand autoCommand = null;
    private MatchType matchType = MatchType.PRACTICE;
    private int matchNumber = 0;
    private Alliance alliance = Alliance.RED_ALLIANCE;
    private double delay = 0.0;
    private StartPos startPos = StartPos.NEAR;
    private Strategy strategy = Strategy.DO_NOTHING;
    private DoJewel jewelChoice = DoJewel.YES;
    private DoCrypto cryptoChoice = DoCrypto.YES;
    private double driveDistance = 0.0;
    private double driveTime = 0.0;
    private double drivePower = 0.0;

    //
    // Implements FtcOpMode abstract method.
    //

    @Override
    public void initRobot()
    {
        //
        // Initializing robot objects.
        //
        robot = new Robot(TrcRobot.RunMode.AUTO_MODE);
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
            case FULL_AUTO:
                autoCommand = new CmdAutoFull(
                        robot, alliance, delay, startPos, jewelChoice == DoJewel.YES,
                        cryptoChoice == DoCrypto.YES);
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

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode()
    {
        robot.tracer.traceInfo(moduleName, "%s: ***** Starting autonomous *****", new Date());
        robot.startMode(TrcRobot.RunMode.AUTO_MODE);
        robot.battery.setEnabled(true);
        robot.dashboard.clearDisplay();
    }   //startMode

    @Override
    public void stopMode()
    {
        robot.stopMode(TrcRobot.RunMode.AUTO_MODE);
        robot.battery.setEnabled(false);
        printPerformanceMetrics(robot.tracer);

        if (USE_TRACELOG)
        {
            robot.tracer.closeTraceLog();
        }
    }   //stopMode

    @Override
    public void runContinuous(double elapsedTime)
    {
        if (autoCommand != null)
        {
            autoCommand.cmdPeriodic(elapsedTime);
        }
    }   //runContinuous

    private void doMenus()
    {
        //
        // Create menus.
        //
        FtcChoiceMenu<MatchType> matchTypeMenu = new FtcChoiceMenu<>("Match type:", null, robot);
        FtcValueMenu matchNumberMenu = new FtcValueMenu(
                "Match number:", matchTypeMenu, robot,
                1.0, 50.0, 1.0, 1.0, "%.0f");
        FtcChoiceMenu<Alliance> allianceMenu = new FtcChoiceMenu<>("Alliance:", matchNumberMenu, robot);
        FtcValueMenu delayMenu = new FtcValueMenu(
                "Delay time:", allianceMenu, robot,
                0.0, 30.0, 1.0, 0.0, " %.0f sec");
        FtcChoiceMenu<StartPos> startPositionMenu = new FtcChoiceMenu<>("Start Position:", delayMenu, robot);
        FtcChoiceMenu<Strategy> strategyMenu = new FtcChoiceMenu<>("Strategies:", startPositionMenu, robot);
        FtcChoiceMenu<DoJewel> jewelMenu = new FtcChoiceMenu<>("Do Jewel?", strategyMenu, robot);
        FtcChoiceMenu<DoCrypto> cryptoMenu = new FtcChoiceMenu<>("Do Crypto?", jewelMenu, robot);
        FtcValueMenu driveDistanceMenu = new FtcValueMenu(
                "Distance:", strategyMenu, robot,
                -12.0, 12.0, 0.5, 4.0, " %.0f ft");
        FtcValueMenu driveTimeMenu = new FtcValueMenu(
                "Drive time:", strategyMenu, robot, 0.0, 30.0, 1.0, 5.0, " %.0f sec");
        FtcValueMenu drivePowerMenu = new FtcValueMenu(
                "Drive power:", strategyMenu, robot, -1.0, 1.0, 0.1, 0.5, " %.1f");

        matchNumberMenu.setChildMenu(allianceMenu);
        delayMenu.setChildMenu(startPositionMenu);
        driveTimeMenu.setChildMenu(drivePowerMenu);

        //
        // Populate choice menus.
        //
        matchTypeMenu.addChoice("Practice", MatchType.PRACTICE, true, matchNumberMenu);
        matchTypeMenu.addChoice("Qualification", MatchType.QUALIFICATION, false, matchNumberMenu);
        matchTypeMenu.addChoice("Semi-final", MatchType.SEMI_FINAL, false, matchNumberMenu);
        matchTypeMenu.addChoice("Final", MatchType.FINAL, false, matchNumberMenu);

        allianceMenu.addChoice("Red", Alliance.RED_ALLIANCE, true, delayMenu);
        allianceMenu.addChoice("Blue", Alliance.BLUE_ALLIANCE, false, delayMenu);

        startPositionMenu.addChoice("Near", StartPos.NEAR, true, strategyMenu);
        startPositionMenu.addChoice("Far", StartPos.FAR, false, strategyMenu);

        strategyMenu.addChoice("Full Auto", Strategy.FULL_AUTO, true, jewelMenu);
        strategyMenu.addChoice("Distance Drive", Strategy.DISTANCE_DRIVE, false, driveDistanceMenu);
        strategyMenu.addChoice("Timed Drive", Strategy.TIMED_DRIVE, false, driveTimeMenu);
        strategyMenu.addChoice("Do nothing", Strategy.DO_NOTHING, false);

        jewelMenu.addChoice("Yes", DoJewel.YES, true, cryptoMenu);
        jewelMenu.addChoice("No", DoJewel.NO, false, cryptoMenu);

        cryptoMenu.addChoice("Yes", DoCrypto.YES, true);
        cryptoMenu.addChoice("No", DoCrypto.NO, false);

        //
        // Traverse menus.
        //
        FtcMenu.walkMenuTree(matchTypeMenu, this);
        //
        // Fetch choices.
        //
        matchType = matchTypeMenu.getCurrentChoiceObject();
        matchNumber = (int)matchNumberMenu.getCurrentValue();
        alliance = allianceMenu.getCurrentChoiceObject();
        delay = delayMenu.getCurrentValue();
        startPos = startPositionMenu.getCurrentChoiceObject();
        strategy = strategyMenu.getCurrentChoiceObject();
        jewelChoice = jewelMenu.getCurrentChoiceObject();
        cryptoChoice = cryptoMenu.getCurrentChoiceObject();
        driveDistance = driveDistanceMenu.getCurrentValue();
        driveTime = driveTimeMenu.getCurrentValue();
        drivePower = drivePowerMenu.getCurrentValue();
        //
        // Show choices.
        //
        robot.dashboard.displayPrintf(1, "== Match: %s ==",
                                      matchType.toString() + "_" + matchNumber);
        robot.dashboard.displayPrintf(2, "Auto Strategy: %s", strategyMenu.getCurrentChoiceText());
        robot.dashboard.displayPrintf(3, "Alliance=%s,Delay=%.0f sec", alliance.toString(), delay);
        robot.dashboard.displayPrintf(4, "StartPos=%s", startPos.toString());
        robot.dashboard.displayPrintf(5, "DoJewel=%s,DoCrypto=%s",
                                      jewelChoice.toString(), cryptoChoice.toString());
        robot.dashboard.displayPrintf(6, "Drive: distance=%.0f ft,Time=%.0f,Power=%.1f",
                                      driveDistance, driveTime, drivePower);
    }   //doMenus

}   //class FtcAuto
