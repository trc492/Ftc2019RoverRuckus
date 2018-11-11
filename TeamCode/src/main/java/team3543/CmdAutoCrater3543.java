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
import common.TensorFlowVision;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

public class CmdAutoCrater3543 implements TrcRobot.RobotCommand
{
    private static final boolean debugXPid = true;
    private static final boolean debugYPid = true;
    private static final boolean debugTurnPid = true;

    private static final String moduleName = "CmdAutoCrater3543";

    private Robot3543 robot;
    private AutoCommon.Alliance alliance;
    private double delay;
    private boolean doMineral;
    private boolean doTeamMarker;
    private boolean doTeammateMineral;

    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;
    private double targetX = 0.0;
    private double targetY = 0.0;
    private int retries = 0;
    private double mineralAngle = 0.0;
    private double wallTurnAngle = 0.0;

    CmdAutoCrater3543(Robot3543 robot, AutoCommon.Alliance alliance, double delay,
                      boolean startHung, boolean doMineral, boolean doTeamMarker, boolean doTeammateMineral)
    {
        robot.tracer.traceInfo(moduleName,
                "Alliance=%s,Delay=%.0f,startHung=%s,Mineral=%s,TeamMarker=%s,TeammateMineral=%s",
                alliance, delay, startHung, doMineral, doTeamMarker, doTeammateMineral);

        this.robot = robot;
        this.alliance = alliance;
        this.delay = delay;
        this.doMineral = doMineral;
        this.doTeamMarker = doTeamMarker;
        this.doTeammateMineral = doTeammateMineral;

        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(startHung? State.DO_DELAY: State.GO_TOWARDS_MINERAL);
    }   //CmdAutoCrater3543

    private enum State
    {
        DO_DELAY,
        LOWER_ROBOT,
        UNHOOK_ROBOT,
        PLOW_TO_CRATER,
        GO_TOWARDS_MINERAL,
        SCAN_MINERAL,
        TURN_TO_MINERAL,
        HIT_MINERAL,
        DRIVE_BACK_TO_START,
        TURN_TO_WALL,
        DRIVE_TO_WALL,
        TURN_TO_DEPOT,
        DRIVE_TO_DEPOT,
        DROP_TEAM_MARKER,
        TURN_TO_CRATER,
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
                    // Intentional falling through.
                    //
                case LOWER_ROBOT:
                    //
                    // The robot started hanging on the lander, lower it to the ground.
                    //
                    robot.elevator.setPosition(RobotInfo3543.ELEVATOR_HANGING_HEIGHT, event, 0.0);
                    sm.waitForSingleEvent(event, State.UNHOOK_ROBOT);
                    break;

                case UNHOOK_ROBOT:
                    robot.tracer.traceInfo(moduleName, "Initial heading=%f", robot.driveBase.getHeading());
                    //
                    // The robot is still hooked, need to unhook first.
                    //
                    targetX = 5.0;
                    targetY = 0.0;
                    nextState = doMineral? State.GO_TOWARDS_MINERAL: State.PLOW_TO_CRATER;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.GO_TOWARDS_MINERAL);
                    break;

                case SCAN_MINERAL:
                    //
                    // Is the mineral left, mid or right? Scan mineral with vision.
                    //
                    if (robot.tensorFlowVision != null)
                    {
                        TensorFlowVision.TargetInfo targetInfo =
                                robot.tensorFlowVision == null? null:
                                        robot.tensorFlowVision.getTargetInfo(TensorFlowVision.LABEL_GOLD_MINERAL);

                        if (targetInfo != null)
                        {
                            //
                            // Found gold mineral.
                            //
                            int third = TensorFlowVision.IMAGE_WIDTH / 3;
                            int t2 = third * 2;
                            int xPos = targetInfo.rect.x;
                            if (xPos <= third && xPos >= 0)
                            {
                                // left
                                mineralAngle = -45.0;
                                wallTurnAngle = -45.0;
                            }
                            else if (xPos <= t2 && xPos > third)
                            {
                                // center
                                mineralAngle = 0.0;
                                wallTurnAngle = -90.0;
                            }
                            else if (xPos > t2)
                            {
                                // right
                                mineralAngle = 45.0;
                                wallTurnAngle = -135.0;
                            }
                            robot.tracer.traceInfo(moduleName, "%s[%d]: found gold mineral %s.",
                                    state, retries, targetInfo);
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

                case TURN_TO_MINERAL:
                    targetX = 0.0;
                    targetY = 0.0;
                    robot.targetHeading += mineralAngle;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.HIT_MINERAL);
                    break;

                case HIT_MINERAL:
                    targetX = 0.0;
                    targetY = 22.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.HIT_MINERAL);
                    break;

                case DRIVE_BACK_TO_START:
                    targetX = 0.0;
                    targetY = -22.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.TURN_TO_WALL);
                    break;

                case TURN_TO_WALL:
                    targetX = 0.0;
                    targetY = 0.0;
                    robot.targetHeading += wallTurnAngle;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_WALL);
                    break;

                case DRIVE_TO_WALL:
                    targetX = 0.0;
                    targetY = 54.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.TURN_TO_DEPOT);
                    break;

                case TURN_TO_DEPOT:
                    targetX = 0.0;
                    targetY = 0.0;
                    robot.targetHeading -= 45.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_DEPOT);
                    break;

                case DRIVE_TO_DEPOT:
                    targetX = 0.0;
                    targetY = 36.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DROP_TEAM_MARKER);
                    break;

                case DROP_TEAM_MARKER:
                    //
                    // Release team marker by opening the deployer.
                    //
                    robot.teamMarkerDeployer.open();
                    timer.set(4.0, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_CRATER);
                    break;

                case DRIVE_TO_CRATER:
                    targetX = 0.0;
                    targetY = -80.0;
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

}   //class CmdAutoCrater3543
