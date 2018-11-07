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
import common.CmdSweepMineral;
import common.PixyVision;
import common.RobotInfo;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

class CmdAutoDepot3543_Pixy implements TrcRobot.RobotCommand
{
    private static final boolean debugXPid = true;
    private static final boolean debugYPid = true;
    private static final boolean debugTurnPid = true;

    private static final String moduleName = "CmdAutoDepot3543_Pixy";

    private enum GoldPosition
    {
        LEFT,
        CENTER,
        RIGHT
    }

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
    private CmdSweepMineral cmdSweepMineral = null;

    private State prevState = null;
    private GoldPosition goldPosition = null;

    CmdAutoDepot3543_Pixy(Robot3543 robot, AutoCommon.Alliance alliance, double delay,
                     boolean startHung, boolean doMineral, boolean doTeamMarker)
    {
        robot.tracer.traceInfo(moduleName,
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
        sm.start(startHung? State.DO_DELAY:
                 doMineral? State.GO_TOWARDS_MINERAL:
                 doTeamMarker? State.PLOW_TO_DEPOT: State.DONE);
    }   //CmdAutoDepot3543

    private enum State
    {
        DO_DELAY,
        LOWER_ROBOT,
        DO_PIXY_SCAN,
        UNHOOK_ROBOT,
        TURN_TO_MINERAL,
        PLOW_TO_DEPOT,
        DROP_TEAM_MARKER_ONLY,
        TURN_TOWARDS_CRATER,
        PARK_AT_CRATER,
        GO_TOWARDS_MINERAL,
        ALIGN_MINERAL_SWEEPER,
        ALIGN_ROBOT_WITH_VUFORIA,
        SWEEP_MINERAL,
        DRIVE_TO_MID_WALL,
        TURN_PARALLEL_TO_WALL,
        DRIVE_FROM_MID_WALL_TO_CRATER,
        DRIVE_FROM_MID_WALL_TO_DEPOT,
        KISS_THE_WALL,
        DROP_TEAM_MARKER,
        DRIVE_FROM_DEPOT_TO_CRATER,
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
                    // Intentional falling through.
                    //
                case LOWER_ROBOT:
                    //
                    // The robot started hanging on the lander, lower it to the ground.
                    //
                    if (robot.pixyVision != null)
                    {
                        robot.pixyVision.setCameraEnabled(true);
                    }
                    robot.elevator.setPosition(RobotInfo3543.ELEVATOR_HANGING_HEIGHT, event, 0.0);
                    sm.waitForSingleEvent(event, State.DO_PIXY_SCAN);
                    break;

                case DO_PIXY_SCAN:
                    //
                    // Is the mineral left, mid or right? Scan the, with the pixy.
                    //
                    if (robot.pixyVision != null)
                    {
                        PixyVision.TargetInfo targetInfo = robot.pixyVision.getTargetInfo(RobotInfo.PIXY_GOLD_MINERAL_SIGNATURE);
                        int third = PixyVision.PIXYCAM_WIDTH / 3;
                        int t2 = third * 2;
                        int xPos = targetInfo.rect.x;
                        if (xPos <= third && xPos >= 0)
                        {
                            // left
                            goldPosition = GoldPosition.LEFT;
                        }
                        else if (xPos <= t2 && xPos > third)
                        {
                            // center
                            goldPosition = GoldPosition.CENTER;
                        }
                        else if (xPos > t2)
                        {
                            // right
                            goldPosition = GoldPosition.RIGHT;
                        }
                        robot.tracer.traceInfo(moduleName, "Gold Position: %s", goldPosition);
                    }
                    sm.waitForSingleEvent(event, State.UNHOOK_ROBOT);
                    break;

                case UNHOOK_ROBOT:
                    robot.tracer.traceInfo(moduleName, "Initial heading=%f", robot.driveBase.getHeading());
                    //
                    // The robot is still hooked, need to unhook first.
                    // THIS NEEDS CHANGING TO MAKE SURE THE ANGLES WORK
                    //
                    targetX = 5.0;
                    targetY = 0.0;
                    nextState = doMineral ? State.TURN_TO_MINERAL: doTeamMarker? State.PLOW_TO_DEPOT: State.DONE;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case TURN_TO_MINERAL:
                    if (goldPosition == GoldPosition.LEFT)
                    {
                        robot.targetHeading -= 45.0;
                    }
                    else if (goldPosition == GoldPosition.RIGHT)
                    {
                        robot.targetHeading += 45.0;
                    }
                    targetX = 0.0;
                    targetY = 0.0;

                    break;

                case PLOW_TO_DEPOT:
                    //
                    // Plow through the middle mineral to the depot.
                    //
                    targetX = 0.0;
                    targetY = 48.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DROP_TEAM_MARKER_ONLY);
                    break;

                case DROP_TEAM_MARKER_ONLY:
                    //
                    // Release team marker by opening the deployer.
                    //
                    robot.teamMarkerDeployer.open();
                    timer.set(4.0, event);
                    sm.waitForSingleEvent(event, State.TURN_TOWARDS_CRATER);
                    break;

                case TURN_TOWARDS_CRATER:
                    targetX = 0.0;
                    targetY = 0.0;
                    robot.targetHeading += 45.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.PARK_AT_CRATER);
                    break;

                case PARK_AT_CRATER:
                    targetX = 0.0;
                    targetY = -72.0;
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
            robot.tracer.traceInfo("Battery", "Voltage=%5.2fV (%5.2fV)",
                    robot.battery.getVoltage(), robot.battery.getLowestVoltage());
            robot.tracer.traceInfo("Raw Encoder",
                    "lf=%.0f, rf=%.0f, lr=%.0f, rr=%.0f",
                    robot.leftFrontWheel.getPosition(),
                    robot.rightFrontWheel.getPosition(),
                    robot.leftRearWheel.getPosition(),
                    robot.rightRearWheel.getPosition());

            if (debugXPid)
            {
                robot.encoderXPidCtrl.printPidInfo(robot.tracer, elapsedTime);
            }

            if (debugYPid)
            {
                robot.encoderYPidCtrl.printPidInfo(robot.tracer, elapsedTime);
            }

            if (debugTurnPid)
            {
                robot.gyroPidCtrl.printPidInfo(robot.tracer, elapsedTime);
            }
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdAutoDepot3543
