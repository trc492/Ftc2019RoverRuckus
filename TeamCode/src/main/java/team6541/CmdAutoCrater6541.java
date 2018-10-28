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

import common.AutoCommon;
import common.CmdSweepMineral;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

class CmdAutoCrater6541 implements TrcRobot.RobotCommand
{
    private static final boolean debugXPid = true;
    private static final boolean debugYPid = true;
    private static final boolean debugTurnPid = true;

    private static final String moduleName = "CmdAutoCrater6541";

    private Robot6541 robot;
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
    private CmdSweepMineral cmdSweepMineral = null;

    CmdAutoCrater6541(Robot6541 robot, AutoCommon.Alliance alliance, double delay,
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
        sm.start(startHung? State.LOWER_ROBOT: State.GO_TOWARDS_MINERAL);
    }   //CmdAutoCrater6541

    private enum State
    {
        LOWER_ROBOT,
        UNHOOK_ROBOT,
        GO_TOWARDS_MINERAL,
        ALIGN_MINERAL_SWEEPER,
        ALIGN_ROBOT_WITH_VUFORIA,
        SWEEP_MINERAL,
        DRIVE_TO_MID_WALL,
        TURN_PARALLEL_TO_WALL,
        DRIVE_FROM_MID_WALL_TO_CRATER,
        DO_DELAY,
        DRIVE_FROM_MID_WALL_TO_DEPOT,
        DROP_TEAM_MARKER,
        DRIVE_FROM_DEPOT_TO_CRATER,
        BACK_OFF_FROM_DEPOT,
        ROTATE_TO_TEAMMATE_MINERALS,
        SWEEP_TEAMMATE_MINERAL,
        DRIVE_BACK_TO_MID_WALL_AFTER_TEAMMATE_MINERAL,
        TURN_TO_CRATER_AFTER_TEAMMATE_MINERAL,
        DRIVE_TO_CRATER_AFTER_TEAMMATE_MINERAL,
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
                case LOWER_ROBOT:
                    //
                    // The robot started hanging on the lander, lower it to the ground.
                    //
                    robot.elevator.setPosition(RobotInfo6541.ELEVATOR_MAX_HEIGHT, event, 0.0);
                    sm.waitForSingleEvent(event, State.UNHOOK_ROBOT);
                    break;

                case UNHOOK_ROBOT:
                    //
                    // The robot is still hooked, need to unhook first.
                    //
                    robot.elevator.openHook();
                    timer.set(0.3, event);
                    sm.waitForSingleEvent(event, State.GO_TOWARDS_MINERAL);
                    break;

                case GO_TOWARDS_MINERAL:
                    //
                    // Move closer to the mineral so the sweeper can reach them.
                    //
                    targetX = 0.0;
                    targetY = 12.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.ALIGN_MINERAL_SWEEPER);
                    break;

                case ALIGN_MINERAL_SWEEPER:
                    //
                    // Turn robot sideway so the sweeper is facing mineral.
                    //
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
                        nextState = State.SWEEP_MINERAL;
                        cmdSweepMineral = new CmdSweepMineral(robot, 0.0);  //TODO: may need to adjust startingY
                    }
                    else
                    {
                        nextState = State.DRIVE_TO_MID_WALL;
                        cmdSweepMineral = null;
                    }
                    sm.setState(nextState);
                    //
                    // Intentionally fall through to the next state to save one cycle time.
                    //
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
                    targetX = 0.0;
                    targetY = -36.0;
                    //
                    // cmdSweepMineral may be null if doMineral is false.
                    //
                    if (cmdSweepMineral != null)
                    {
                        //
                        // Adjust the distance by how far we went for sweeping mineral.
                        //
                        targetY -= cmdSweepMineral.getDistanceYTravelled();
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
                    nextState = !doTeamMarker? State.DRIVE_FROM_MID_WALL_TO_CRATER :
                                delay != 0.0? State.DO_DELAY: State.DRIVE_FROM_MID_WALL_TO_DEPOT;
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case DRIVE_FROM_MID_WALL_TO_CRATER:
                    //
                    // Go direct to crater and park there.
                    //
                    targetX = 0.0;
                    targetY = 24.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DO_DELAY:
                    //
                    // Wait for alliance partner to clear the path.
                    //
                    timer.set(delay, event);
                    sm.waitForSingleEvent(event, State.DRIVE_FROM_MID_WALL_TO_DEPOT);
                    break;

                case DRIVE_FROM_MID_WALL_TO_DEPOT:
                    //
                    // Drive to depot to drop off team marker.
                    //
                    targetX = 0.0;
                    targetY = -30.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DROP_TEAM_MARKER);
                    break;

                case DROP_TEAM_MARKER:
                    //
                    // Release team marker by opening the deployer.
                    //
                    robot.teamMarkerDeployer.open();
                    timer.set(0.3, event);
                    nextState = !doTeammateMineral ? State.DRIVE_FROM_DEPOT_TO_CRATER: State.BACK_OFF_FROM_DEPOT;
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case DRIVE_FROM_DEPOT_TO_CRATER:
                    //
                    // Drive back to the crater and park there.
                    //
                    targetX = 0.0;
                    targetY = -72.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case BACK_OFF_FROM_DEPOT:
                    //
                    // Back off from the depot a little so we don't trample on our team marker and better aligned
                    // with our teammate's mineral.
                    //
                    targetX = 0.0;
                    targetY = 10.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.ROTATE_TO_TEAMMATE_MINERALS);
                    break;

                case ROTATE_TO_TEAMMATE_MINERALS:
                    //
                    // Align sweeper with teammates minerals.
                    //
                    targetX = 0.0;
                    targetY = 0.0;
                    robot.targetHeading += 135.0;
                    cmdSweepMineral = new CmdSweepMineral(robot, -22.0);
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.SWEEP_TEAMMATE_MINERAL);
                    break;

                case SWEEP_TEAMMATE_MINERAL:
                    //
                    // Remain in this state until sweeping is done.
                    //
                    if (cmdSweepMineral.cmdPeriodic(elapsedTime))
                    {
                        sm.setState(State.DRIVE_BACK_TO_MID_WALL_AFTER_TEAMMATE_MINERAL);
                    }
                    break;

                case DRIVE_BACK_TO_MID_WALL_AFTER_TEAMMATE_MINERAL:
                    //
                    // Go back to mid wall.
                    //
                    targetX = 0.0;
                    targetY = -cmdSweepMineral.getDistanceYTravelled();
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.TURN_TO_CRATER_AFTER_TEAMMATE_MINERAL);
                    break;

                case TURN_TO_CRATER_AFTER_TEAMMATE_MINERAL:
                    //
                    // Align the robot parallel to wall.
                    //
                    targetX = 0.0;
                    targetY = 0.0;
                    robot.targetHeading += 45.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_CRATER_AFTER_TEAMMATE_MINERAL);
                    break;

                case DRIVE_TO_CRATER_AFTER_TEAMMATE_MINERAL:
                    //
                    // Go to crater and park there.
                    //
                    targetX = 0.0;
                    targetY = -24.0;
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

}   //class CmdAutoCrater6541
