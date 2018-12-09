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

import common.AutoCommon;
import common.CmdDisplaceMineral;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

class CmdAutoDepot3543 implements TrcRobot.RobotCommand
{
    private static final boolean debugXPid = true;
    private static final boolean debugYPid = true;
    private static final boolean debugTurnPid = true;

    private static final String moduleName = "CmdAutoDepot3543";

    private Robot3543 robot;
    private AutoCommon.Alliance alliance;
    private double delay;
    private boolean doMineral;
    private boolean doTeamMarker;

    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;

    private double targetX = 0.0;
    private double targetY = 0.0;
    private TrcRobot.RobotCommand cmdDisplaceMineral = null;

    CmdAutoDepot3543(Robot3543 robot, AutoCommon.Alliance alliance, double delay,
                     boolean startHung, boolean doMineral, boolean doTeamMarker)
    {
        robot.globalTracer.traceInfo(moduleName,
                "Alliance=%s,Delay=%.0f,Hanging=%s,Mineral=%s,TeamMarker=%s",
                alliance, delay, startHung, doMineral, doTeamMarker);

        this.robot = robot;
        this.alliance = alliance;
        this.delay = delay;
        this.doMineral = doMineral;
        this.doTeamMarker = doTeamMarker;

        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(startHung? State.DO_DELAY: doMineral? State.DO_MINERAL: State.PLOW_TO_DEPOT);
    }   //CmdAutoDepot3543

    private enum State
    {
        DO_DELAY,
        LOWER_ROBOT,
        UNHOOK_ROBOT,
        PLOW_TO_DEPOT,
        DO_MINERAL,
        DISPLACE_MINERAL,
        DROP_TEAM_MARKER,
        TURN_TO_CRATER,
        KISS_THE_WALL,
        DRIVE_TO_CRATER,
        DONE
    }   //enum State

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        State state = sm.checkReadyAndGetState();
        State nextState;
        boolean traceState = true;

        if (state == null)
        {
            robot.dashboard.displayPrintf(1, "State: Disabled");
        }
        else
        {
            robot.dashboard.displayPrintf(1, "State: %s", state);

            switch (state)
            {
                case DO_DELAY:
                    //
                    // Do delay if we need to wait for partner to lower their robot first.
                    //
                    if (delay > 0.0)
                    {
                        timer.set(delay, event);
                        sm.waitForSingleEvent(event, State.LOWER_ROBOT);
                        break;
                    }
                    else
                    {
                        sm.setState(State.LOWER_ROBOT);
                    }
                    //
                    // Intentionally falling through.
                    //
                case LOWER_ROBOT:
                    //
                    // The robot started hanging on the lander, lower it to the ground.
                    //
                    robot.elevator.setPosition(RobotInfo3543.ELEVATOR_HANGING_HEIGHT, event, 0.0);
                    sm.waitForSingleEvent(event, State.UNHOOK_ROBOT);
                    break;

                case UNHOOK_ROBOT:
                    //
                    // The robot is still hooked, need to unhook first.
                    //
                    robot.globalTracer.traceInfo(moduleName, "Initial heading=%f", robot.driveBase.getHeading());
                    targetX = RobotInfo3543.UNHOOK_DISPLACEMENT;
                    targetY = 0.0;
                    nextState = doMineral? State.DO_MINERAL: State.PLOW_TO_DEPOT;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case PLOW_TO_DEPOT:
                    //
                    // We are not using vision, so just plow through the middle mineral to the depot.
                    //
                    robot.elevator.setPosition(RobotInfo3543.ELEVATOR_MIN_HEIGHT);
                    targetX = 0.0;
                    targetY = 36.0; // prev: 36
                    nextState = doTeamMarker? State.DROP_TEAM_MARKER: State.TURN_TO_CRATER;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case DO_MINERAL:
                    //
                    // Set up CmdDisplaceMineral to use vision to displace the gold mineral.
                    //
                    robot.elevator.setPosition(RobotInfo3543.ELEVATOR_MIN_HEIGHT);
                    cmdDisplaceMineral = new CmdDisplaceMineral(
                            robot, 3543, true, RobotInfo3543.SIDE_MINERAL_ANGLE,
                            RobotInfo3543.UNHOOK_DISPLACEMENT);
                    sm.setState(State.DISPLACE_MINERAL);
                    //
                    // Intentionally falling through.
                    //
                case DISPLACE_MINERAL:
                    //
                    // Run CmdDisplaceMineral until it's done.
                    //
                    traceState = false;
                    if (cmdDisplaceMineral.cmdPeriodic(elapsedTime))
                    {
                        sm.setState(doTeamMarker? State.DROP_TEAM_MARKER: State.TURN_TO_CRATER);
                    }
                    else
                    {
                        break;
                    }
                    //
                    // Intentionally falling through.
                    //
                case DROP_TEAM_MARKER:
                    //
                    // Release team marker by opening the deployer.
                    //
                    robot.teamMarkerDeployer.open();
                    timer.set(4.0, event);
                    sm.waitForSingleEvent(event, State.TURN_TO_CRATER);
                    break;

                case TURN_TO_CRATER:
                    //
                    // Turn towards the crater.
                    //
                    targetX = 0.0;
                    targetY = 0.0;
                    robot.targetHeading = 45.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.KISS_THE_WALL);
                    break;

                case KISS_THE_WALL:
                    robot.driveBase.holonomicDrive(-0.35, 0.0, 0.0);
                    timer.set(1.6, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_CRATER);
                    break;

                case DRIVE_TO_CRATER:
                    robot.driveBase.stop();
                    //
                    // Go and park at the crater.
                    //
                    targetX = 0.0;
                    targetY = -76.0; // prev: -72 inches
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    sm.stop();
                    break;
            }

            if (traceState)
            {
                robot.traceStateInfo(elapsedTime, state.toString(), targetX, targetY, robot.targetHeading);
            }
        }

        if (robot.pidDrive.isActive())
        {
            if (robot.battery != null)
            {
                robot.globalTracer.traceInfo("Battery", "Voltage=%5.2fV (%5.2fV)",
                        robot.battery.getVoltage(), robot.battery.getLowestVoltage());
            }
            robot.globalTracer.traceInfo("Raw Encoder",
                    "lf=%.0f, rf=%.0f, lr=%.0f, rr=%.0f",
                    robot.leftFrontWheel.getPosition(),
                    robot.rightFrontWheel.getPosition(),
                    robot.leftRearWheel.getPosition(),
                    robot.rightRearWheel.getPosition());

            if (debugXPid)
            {
                robot.encoderXPidCtrl.printPidInfo(robot.globalTracer, elapsedTime);
            }

            if (debugYPid)
            {
                robot.encoderYPidCtrl.printPidInfo(robot.globalTracer, elapsedTime);
            }

            if (debugTurnPid)
            {
                robot.gyroPidCtrl.printPidInfo(robot.globalTracer, elapsedTime);
            }
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdAutoDepot3543
