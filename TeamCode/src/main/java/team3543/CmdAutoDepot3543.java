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
    private CmdSweepMineral cmdSweepMineral = null;

    private int retries = 0;
    private double mineralAngle = 0.0;

    private State prevState = null;

    CmdAutoDepot3543(Robot3543 robot, AutoCommon.Alliance alliance, double delay,
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

        if (robot.pixyVision != null)
        {
            robot.pixyVision.setCameraEnabled(true);
        }

        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(startHung? State.DO_DELAY:
                 doMineral? State.PLOW_TO_MINERAL :
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
        LEFT_MINERAL_TURN_TO_DEPOT,
        LEFT_MINERAL_DRIVE_TO_DEPOT,
        LEFT_MINERAL_DROP_MARKER,
        LEFT_MINERAL_TURN_TO_CRATER,
        LEFT_MINERAL_DRIVE_TO_CRATER,
        MID_MINERAL_DRIVE_TO_DEPOT,
        MID_MINERAL_DROP_MARKER,
        MID_MINERAL_TURN_TO_CRATER,
        MID_MINERAL_DRIVE_TO_CRATER,
        RIGHT_MINERAL_TURN_TO_DEPOT,
        RIGHT_MINERAL_DRIVE_TO_DEPOT,
        RIGHT_MINERAL_DROP_MARKER,
        RIGHT_MINERAL_TURN_TO_CRATER,
        RIGHT_MINERAL_DRIVE_TO_CRATER,
        DROP_TEAM_MARKER_ONLY,
        TURN_TOWARDS_CRATER,
        PARK_AT_CRATER,
        PLOW_TO_MINERAL,
        ALIGN_MINERAL_SWEEPER,
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
                    robot.elevator.setPosition(RobotInfo3543.ELEVATOR_HANGING_HEIGHT, event, 0.0);
                    sm.waitForSingleEvent(event, State.DO_PIXY_SCAN);
                    break;

                case DO_PIXY_SCAN:
                    //
                    // Is the mineral left, mid or right? Scan the, with the pixy.
                    //
                    if (robot.pixyVision != null)
                    {
                        //
                        // CodeReview: Pixy may fail to find the target in which case it will return null.
                        // You need to handle that! This will cause NullPointerException.
                        //
                        PixyVision.TargetInfo targetInfo =
                                robot.pixyVision == null? null:
                                        robot.pixyVision.getTargetInfo(RobotInfo.PIXY_GOLD_MINERAL_SIGNATURE);

                        if (targetInfo != null)
                        {
                            //
                            // Found gold mineral.
                            //
                            int third = PixyVision.PIXYCAM_WIDTH / 3;
                            int t2 = third * 2;
                            int xPos = targetInfo.rect.x;
                            if (xPos <= third && xPos >= 0)
                            {
                                // left
                                mineralAngle = -45.0;
                            }
                            else if (xPos <= t2 && xPos > third)
                            {
                                // center
                                mineralAngle = 0.0;
                            }
                            else if (xPos > t2)
                            {
                                // right
                               mineralAngle = 45.0;
                            }
                            robot.tracer.traceInfo(moduleName, "%s[%d]: found gold mineral (x/y/angle/rect) %s.",
                                    state, retries,
                                    targetInfo.xDistance, targetInfo.yDistance, targetInfo.angle, targetInfo.rect);
                            sm.setState(State.UNHOOK_ROBOT);
                        }
                        else
                        {
                            //
                            // Don't see gold mineral.
                            //
                            retries++;
                            if (retries < 3)
                            {
                                //
                                // Gold not found, remain in this state and try again.
                                //
                                robot.tracer.traceInfo(moduleName, "%s[%d]: gold mineral not found, try again.",
                                        state, retries);
                                break;
                            }
                            else
                            {
                                mineralAngle = 0.0;
                                sm.setState(State.UNHOOK_ROBOT);
                            }
                        }
                    }
                    //
                    // Intentionally falling through.
                    //

                case UNHOOK_ROBOT:
                    robot.tracer.traceInfo(moduleName, "Initial heading=%f", robot.driveBase.getHeading());
                    //
                    // The robot is still hooked, need to unhook first.
                    // THIS NEEDS CHANGING TO MAKE SURE THE ANGLES WORK
                    //
                    targetX = 5.0;
                    targetY = 0.0;
                    nextState = State.TURN_TO_MINERAL;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case TURN_TO_MINERAL:
                    robot.targetHeading += mineralAngle;
                    targetX = 0.0;
                    targetY = 0.0;

                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.PLOW_TO_MINERAL);
                    break;

                case PLOW_TO_MINERAL:
                    //
                    // Move closer to the mineral so the sweeper can reach them.
                    //
                    // NEEDS TUNING LATER
                    targetX = 0.0;
                    targetY = 22.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    if (mineralAngle == -45.0)
                    {
                        nextState = State.LEFT_MINERAL_TURN_TO_DEPOT;
                    }
                    else if (mineralAngle == 0.0)
                    {
                        nextState = State.MID_MINERAL_TURN_TO_CRATER;
                    }
                    else//if (mineralAngle == 45.0)
                    {
                        nextState = State.RIGHT_MINERAL_TURN_TO_DEPOT;
                    }
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case LEFT_MINERAL_TURN_TO_DEPOT:
                    targetX = 0.0;
                    targetY = 0.0;
                    robot.targetHeading += 90.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.LEFT_MINERAL_DRIVE_TO_DEPOT);
                    break;

                case LEFT_MINERAL_DRIVE_TO_DEPOT:
                    targetX = 0.0;
                    targetY = 36.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.LEFT_MINERAL_DROP_MARKER);
                    break;

                case LEFT_MINERAL_DROP_MARKER:
                    robot.teamMarkerDeployer.open();
                    timer.set(4.0, event);
                    sm.waitForSingleEvent(event, State.LEFT_MINERAL_TURN_TO_CRATER);
                    break;

                case LEFT_MINERAL_DRIVE_TO_CRATER:
                    targetX = 0.0;
                    targetY = 72.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case MID_MINERAL_DRIVE_TO_DEPOT:
                    targetX = 0.0;
                    targetY = 38.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.MID_MINERAL_DROP_MARKER);
                    break;

                case MID_MINERAL_DROP_MARKER:
                    robot.teamMarkerDeployer.open();
                    timer.set(4.0, event);
                    sm.waitForSingleEvent(event, State.MID_MINERAL_TURN_TO_CRATER);
                    break;

                case MID_MINERAL_TURN_TO_CRATER:
                    targetX = 0.0;
                    targetY = 0.0;
                    robot.targetHeading += 45.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.MID_MINERAL_DRIVE_TO_CRATER);
                    break;

                case MID_MINERAL_DRIVE_TO_CRATER:
                    targetX = 0.0;
                    targetY = 72.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case RIGHT_MINERAL_TURN_TO_DEPOT:
                    targetX = 0.0;
                    targetY = 0.0;
                    robot.targetHeading -= 90.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.RIGHT_MINERAL_DRIVE_TO_DEPOT);
                    break;

                case RIGHT_MINERAL_DRIVE_TO_DEPOT:
                    targetX = 0.0;
                    targetY = 36.0; // Needs tuning later (>24in. always)
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.RIGHT_MINERAL_DROP_MARKER);
                    break;

                case RIGHT_MINERAL_DROP_MARKER:
                    robot.teamMarkerDeployer.open();
                    timer.set(4.0, event);
                    sm.waitForSingleEvent(event, State.RIGHT_MINERAL_TURN_TO_CRATER);
                    break;

                case RIGHT_MINERAL_TURN_TO_CRATER:
                    targetX = 0.0;
                    targetY = 0.0;
                    robot.targetHeading -= 90.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.RIGHT_MINERAL_DRIVE_TO_CRATER);
                    break;

                case RIGHT_MINERAL_DRIVE_TO_CRATER:
                    targetX = 0.0;
                    targetY = 72.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
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
