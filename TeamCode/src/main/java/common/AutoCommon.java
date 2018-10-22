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

import java.util.Date;

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcOpMode;
import ftclib.FtcValueMenu;
import trclib.TrcRobot;

public abstract class AutoCommon extends FtcOpMode
{
    protected static final boolean USE_TRACELOG = true;

    public enum MatchType
    {
        PRACTICE,
        QUALIFICATION,
        SEMI_FINAL,
        FINAL
    }   //enum MatchType

    public enum Alliance
    {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }   //enum Alliance

    public enum Strategy
    {
        CRATER_AUTO,
        DEPOT_AUTO,
        DISTANCE_DRIVE,
        TIMED_DRIVE,
        DO_NOTHING
    }   //enum Strategy

    private final String moduleName;
    private Robot robot;
    protected TrcRobot.RobotCommand autoCommand = null;
    protected MatchType matchType = MatchType.PRACTICE;
    protected int matchNumber = 0;
    protected Alliance alliance = Alliance.RED_ALLIANCE;
    protected double delay = 0.0;
    protected Strategy strategy = Strategy.CRATER_AUTO;
    protected double driveDistance = 0.0;
    protected double driveTime = 0.0;
    protected double drivePower = 0.0;

    protected boolean isHanging = false;
    protected boolean doMineral = false;
    protected boolean doTeamMarker = false;
    protected boolean doTeammateMineral = false;

    public AutoCommon(String moduleName)
    {
        this.moduleName = moduleName;
    }   //AutoCommon

    public void setRobot(Robot robot)
    {
        this.robot = robot;
    }   //setRobot

    //
    // Implements FtcOpMode abstract method.
    //

    @Override
    public void startMode(TrcRobot.RunMode runMode)
    {
        robot.tracer.traceInfo(moduleName, "%s: ***** Starting autonomous *****", new Date());
        robot.startMode(TrcRobot.RunMode.AUTO_MODE);
        robot.battery.setTaskEnabled(true);
        robot.dashboard.clearDisplay();
    }   //startMode

    @Override
    public void stopMode(TrcRobot.RunMode runMode)
    {
        robot.stopMode(TrcRobot.RunMode.AUTO_MODE);
        robot.battery.setTaskEnabled(false);
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

    protected void doMenus()
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
        FtcChoiceMenu<Strategy> strategyMenu = new FtcChoiceMenu<>("Strategies:", delayMenu, robot);
        FtcValueMenu driveDistanceMenu = new FtcValueMenu(
                "Distance:", strategyMenu, robot,
                -12.0, 12.0, 0.5, 4.0, " %.0f ft");
        FtcValueMenu driveTimeMenu = new FtcValueMenu(
                "Drive time:", strategyMenu, robot,
                0.0, 30.0, 1.0, 5.0, " %.0f sec");
        FtcValueMenu drivePowerMenu = new FtcValueMenu(
                "Drive power:", strategyMenu, robot,
                -1.0, 1.0, 0.1, 0.5, " %.1f");

        matchNumberMenu.setChildMenu(allianceMenu);
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

        strategyMenu.addChoice("Crater Auto", Strategy.CRATER_AUTO, true);
        strategyMenu.addChoice("Depot Auto", Strategy.DEPOT_AUTO, false);
        strategyMenu.addChoice("Distance Drive", Strategy.DISTANCE_DRIVE, false, driveDistanceMenu);
        strategyMenu.addChoice("Timed Drive", Strategy.TIMED_DRIVE, false, driveTimeMenu);
        strategyMenu.addChoice("Do nothing", Strategy.DO_NOTHING, false);
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
        strategy = strategyMenu.getCurrentChoiceObject();
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
        robot.dashboard.displayPrintf(4, "Drive: distance=%.0f ft,Time=%.0f,Power=%.1f",
                driveDistance, driveTime, drivePower);
        robot.dashboard.displayPrintf(5, "Hanging=%s,Mineral=%s,TeamMarker=%s,TeammateMineral=%s",
                isHanging, doMineral, doTeamMarker, doTeammateMineral);
    }   //doMenus

}   //class AutoCommon