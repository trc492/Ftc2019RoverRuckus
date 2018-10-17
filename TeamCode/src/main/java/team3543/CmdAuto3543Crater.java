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

import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

class CmdAuto3543Crater implements TrcRobot.RobotCommand
{
    private static final boolean debugXPid = true;
    private static final boolean debugYPid = true;
    private static final boolean debugTurnPid = true;

    private static final String moduleName = "CmdAuto3543Crater";

    private Robot3543 robot;
    private FtcAuto3543.Alliance alliance;
    private double delay;
    private boolean isHanging;
    private boolean doMineral;
    private boolean doTeamMarker;
    private boolean doOtherTeamMineral;
    private FtcAuto3543.Park park;

    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;
    private double targetX = 0.0;
    private double targetY = 0.0;

    CmdAuto3543Crater(Robot3543 robot, FtcAuto3543.Alliance alliance, double delay,
                      boolean isHanging, boolean doMineral, boolean doTeamMarker, boolean doOtherTeamMineral,
                      FtcAuto3543.Park park)
    {
        robot.tracer.traceInfo(moduleName,
                "Alliance=%s,Delay=%.0f,Hanging=%s,Mineral=%s,TeamMarker=%s,2ndTeamMineral=%s,Park=%s",
                alliance, delay, isHanging, doMineral, doTeamMarker, doOtherTeamMineral, park);

        this.robot = robot;
        this.alliance = alliance;
        this.delay = delay;
        this.isHanging = isHanging;
        this.doMineral = doMineral;
        this.doTeamMarker = doTeamMarker;
        this.doOtherTeamMineral = doOtherTeamMineral;
        this.park = park;

        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.DO_DELAY);
    }   //CmdAuto3543Crater

    private enum State
    {
        LOWER_ROBOT,
        ALIGN_ROBOT_WITH_VUFORIA,
        DO_DELAY,
        GO_FORWARD_A_BIT,
        ROTATE_ROBOT_90_DEGREES_TO_LEFT,
        GET_MINERAL_POSITION_WITH_PIXY,
        ALIGN_WITH_MINERAL,
        DEPLOY_MINERAL_SWEEPER,
        DISPLACE_MINERAL,
        RETRACT_MINERAL_SWEEPER,
        DRIVE_FORWARD_TO_MIDPOINT,
        TURN_PARALLEL_TO_WALL,
        DRIVE_BACKWARDS_INTO_CRATER,
        DRIVE_FORWARD_TO_DEPOT,
        DROP_TEAM_MARKER,
        BACK_UP_FOR_TEAMMATE_MINERAL,
        ROTATE_TO_TEAMMATE_MINERALS,
        DRIVE_FORWARD_SOME_DISTANCE_FOR_TEAMMATE_MINERAL,
        TURN_TO_CRATER_AFTER_TEAMMATE_MINERAL,
        DRIVE_FORWARD_TO_CRATER,
        DRIVE_BACKWARD_TO_CRATER,
        DONE
    }   //enum State

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        State state = sm.checkReadyAndGetState();

        if (state == null)
        {
            robot.dashboard.displayPrintf(1, "State: Disabled");
        }
        else
        {
            robot.dashboard.displayPrintf(1, "State: %s", state);
            /*
             Lower the robot.
             Align robot using Vuforia Image to face the center.

             Go forward a bit (NEED MEASURING LATER)
             Rotate the robot 90 degrees to the left, using the Vuforia Image as reference for heading.
             Get mineral position with Pixy cam.
             Drive forward or backward to align with the mineral.
             Deploy the mineral sweeper.
             Drive forwards to displace mineral.
             Retract the mineral sweeper.
             Drive forward to marked X.
             Turn 45 degrees to the left.

             Displace mineral ONLY?

             if Yes:
             Drive backwards into crater.
             THE END (for AUTONOMOUS)

             if No:
             Drive foward to the depot.
             Drop the team marker.

             Do teammate's mineral?

             if Yes:
             Back up some distance. (NEED MEASURING LATER)
             Rotate 135 degrees to the right.
             Get mineral position with Pixy cam.
             Drive forward or backward to align with the mineral.
             Drive forwards to displace mineral.
             Retract the mineral sweeper.
             Drive forward for some distance. (NEED MEASURING LATER)
             Turn 45 degrees to the right.
             Drive forward to crater.
             THE END (for AUTONOMOUS)

             if No:
             Drive backwards into crater.
             THE END (for AUTONOMOUS)

             */
            switch (state)
            {
                case LOWER_ROBOT:
                    break;

                case ALIGN_ROBOT_WITH_VUFORIA:
                    break;

                case DO_DELAY:
                    //
                    // Do delay if any.
                    //
                    robot.tracer.traceInfo(state.toString(), "Delay=%.0f", delay);
                    if (delay == 0.0)
                    {
                        sm.setState(State.DONE);
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.waitForSingleEvent(event, State.DONE);
                    }
                    break;

                case GO_FORWARD_A_BIT:
                    break;

                case ROTATE_ROBOT_90_DEGREES_TO_LEFT:
                    break;

                case GET_MINERAL_POSITION_WITH_PIXY:
                    break;

                case ALIGN_WITH_MINERAL:
                    break;

                case DEPLOY_MINERAL_SWEEPER:
                    break;

                case DISPLACE_MINERAL:
                    break;

                case RETRACT_MINERAL_SWEEPER:
                    break;

                case DRIVE_FORWARD_TO_MIDPOINT:
                    break;

                case TURN_PARALLEL_TO_WALL:
                    break;

                case DRIVE_BACKWARDS_INTO_CRATER:
                    break;

                case DRIVE_FORWARD_TO_DEPOT:
                    break;

                case DROP_TEAM_MARKER:
                    break;

                case BACK_UP_FOR_TEAMMATE_MINERAL:
                    break;

                case ROTATE_TO_TEAMMATE_MINERALS:
                    break;

                case DRIVE_FORWARD_SOME_DISTANCE_FOR_TEAMMATE_MINERAL:
                    break;

                case TURN_TO_CRATER_AFTER_TEAMMATE_MINERAL:
                    break;

                case DRIVE_FORWARD_TO_CRATER:
                    break;

                case DRIVE_BACKWARD_TO_CRATER:
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    sm.stop();
                    break;
            }
            robot.traceStateInfo(elapsedTime, state.toString(), targetX, targetY, robot.targetHeading);
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

}   //class CmdAuto3543Crater
