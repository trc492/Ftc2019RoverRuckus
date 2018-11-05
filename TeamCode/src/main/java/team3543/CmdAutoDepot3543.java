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
        UNHOOK_ROBOT,
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
                    nextState = doMineral ? State.GO_TOWARDS_MINERAL: doTeamMarker? State.PLOW_TO_DEPOT: State.DONE;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, nextState);
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

                case GO_TOWARDS_MINERAL:
                    //
                    // Move closer to the mineral so the sweeper can reach them.
                    //
                    targetX = 0.0;
                    targetY = 22.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.ALIGN_MINERAL_SWEEPER);
                    break;

                case ALIGN_MINERAL_SWEEPER:
                    //
                    // Retract elevator.
                    // Turn robot sideway so the sweeper is facing mineral.
                    //
                    robot.elevator.setPosition(RobotInfo3543.ELEVATOR_MIN_HEIGHT);
                    if (robot.vuforiaVision != null)
                    {
                        robot.vuforiaVision.setEnabled(true);
                    }
                    targetX = 0.0;
                    targetY = 0.0;
                    robot.targetHeading += 90.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.ALIGN_ROBOT_WITH_VUFORIA);
                    break;

                case ALIGN_ROBOT_WITH_VUFORIA:
                    //
                    // Align the robot heading with robot orientation reported by Vuforia.
                    // In case Vuforia failed to see images, set the default orientation accordingly.
                    //
                    robot.alignHeadingWithVuforia(alliance == AutoCommon.Alliance.RED_ALLIANCE? -45.0: 135.0);
                    if (doMineral)
                    {
                        //
                        // Go to the starting position before engaging mineral sweeping.
                        // If Pixy camera is active, we will start with the right most mineral. If not, we will always
                        // sweep the left most mineral and hope it's the correct one.
                        //
                        double startingY = robot.pixyVision == null? -12.0: 18.0;
                        cmdSweepMineral = new CmdSweepMineral(robot, startingY);
                        //
                        // Vision generally will impact performance, so we only enable it if it's needed.
                        //
                        if (robot.pixyVision != null)
                        {
                            robot.pixyVision.setCameraEnabled(true);
                        }
                        targetX = 0.0;
                        targetY = startingY;
                        robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                        sm.waitForSingleEvent(event, State.SWEEP_MINERAL);
                    }
                    else
                    {
                        cmdSweepMineral = null;
                        sm.setState(State.DRIVE_TO_MID_WALL);
                    }
                    break;

                case SWEEP_MINERAL:
                    //
                    // Remain in this state until sweeping is done.
                    //
                    if (cmdSweepMineral.cmdPeriodic(elapsedTime))
                    {
                        sm.setState(State.DRIVE_TO_MID_WALL);
                    }
                    traceState = false;
                    break;

                case DRIVE_TO_MID_WALL:
                    //
                    // Drive towards mid wall.
                    //
                    if (robot.pixyVision != null)
                    {
                        robot.pixyVision.setCameraEnabled(false);
                    }

                    if (robot.vuforiaVision != null)
                    {
                        robot.vuforiaVision.setEnabled(false);
                    }

                    targetX = 0.0;
                    targetY = -56.0;
                    //
                    // cmdSweepMineral may be null if doMineral is false.
                    //
                    if (cmdSweepMineral != null)
                    {
                        //
                        // Adjust the distance by how far we went for sweeping mineral.
                        //
                        double distanceYTravelled = cmdSweepMineral.getDistanceYTravelled();
                        targetY -= distanceYTravelled;
                        robot.tracer.traceInfo(moduleName, "yTravelled=%.1f, yTarget=%.1f",
                                distanceYTravelled, targetY);
                        cmdSweepMineral = null;
                    }
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.TURN_PARALLEL_TO_WALL);
                    break;

                case TURN_PARALLEL_TO_WALL:
                    //
                    // Align the robot parallel to the wall.
                    //
                    targetX = 0.0;
                    targetY = 0.0;
                    robot.targetHeading -= 45.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    prevState = State.TURN_PARALLEL_TO_WALL;
                    nextState = !doTeamMarker? State.DRIVE_FROM_MID_WALL_TO_CRATER: State.KISS_THE_WALL;
                    // nextState = State.DONE;
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case DRIVE_FROM_MID_WALL_TO_CRATER:
                    //
                    // Go direct to crater and park there.
                    //
                    targetX = 0.0;
                    targetY = -24.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case KISS_THE_WALL:
                    robot.driveBase.holonomicDrive(-0.35, 0.0, 0.0);
                    timer.set(1.6, event);

                    if (prevState == State.TURN_PARALLEL_TO_WALL)
                    {
                        sm.waitForSingleEvent(event, State.DRIVE_FROM_MID_WALL_TO_DEPOT);
                    }
                    else if (prevState == State.DROP_TEAM_MARKER)
                    {
                        sm.waitForSingleEvent(event, State.DRIVE_FROM_DEPOT_TO_CRATER);
                    }
                    break;

                case DRIVE_FROM_MID_WALL_TO_DEPOT:
                    //
                    // Drive to depot to drop off team marker.
                    //
                    robot.driveBase.stop();
                    // robot.targetHeading = 0.0;
                    targetX = 0.0; // prev: -4.0
                    targetY = 50.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DROP_TEAM_MARKER);
                    break;

                case DROP_TEAM_MARKER:
                    //
                    // Release team marker by opening the deployer.
                    //
                    prevState = State.DROP_TEAM_MARKER;
                    robot.teamMarkerDeployer.open();
                    timer.set(4.0, event);
                    sm.waitForSingleEvent(event, State.KISS_THE_WALL);
                    break;

                case DRIVE_FROM_DEPOT_TO_CRATER:
                    //
                    // Drive back to the crater and park there.
                    //
                    robot.driveBase.stop();
                    targetX = 0.0;
                    targetY = -84.0;
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
