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

import team3543.Robot3543;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

public class CmdSweepMineral implements TrcRobot.RobotCommand
{
    enum State
    {
        START,
        SCAN_MINERALS,
        EXTEND_MINERAL_SWEEPER,
        DISPLACE_MINERAL,
        RETRACT_MINERAL_SWEEPER,
        DONE
    }

    private static final boolean debugXPid = true;
    private static final boolean debugYPid = true;
    private static final boolean debugTurnPid = true;

    private static final String moduleName = "CmdSweepMineral";

    private Robot3543 robot;
    private double startingY;

    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;

    private double targetX = 0.0;
    private double targetY = 0.0;

    private int retries = 0;

    private double distanceTravelled = 0.0;

    // distance: 14.5 inches between minerals

    //
    // Starting position 0 inch : 25 inch backuo


    public CmdSweepMineral(Robot3543 robot, double startingY)
    {
        this.robot = robot;
        this.startingY = startingY;

        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START);
    }

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {

        State state = sm.checkReadyAndGetState();
        State nextState;

        if (state == null)
        {
            robot.dashboard.displayPrintf(1, "State: Disabled");
        }
        else
        {
            robot.dashboard.displayPrintf(1, "State: %s", state);
            switch (state)
            {
                case START:
                    targetX = 0.0;
                    // move robot 25 inches off the center. (assume we are in the middle)
                    setTargetY(-startingY - 25.0);
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case SCAN_MINERALS:
                    targetX = 0.0;
                    PixyVision.TargetInfo targetInfo =
                            robot.pixyVision.getTargetInfo(Robot.PIXY_GOLD_MINERAL_SIGNATURE);
                    if (targetInfo != null)
                    {
                        //
                        // Get the sweeper right behind the gold mineral. The pixy camera is 4 inches behind the arm.
                        // We will give another 4 inches margin so a total of 8 inches.
                        //
                        setTargetY(targetInfo.xDistance - 8.0);
                        nextState = State.EXTEND_MINERAL_SWEEPER;
                    }
                    else
                    {
                        // this is the distance between two minerals
                        setTargetY(14.5);
                        retries++;
                        if (retries == 3)
                        {
                            nextState = State.EXTEND_MINERAL_SWEEPER;
                        }
                        else
                        {
                            nextState = State.SCAN_MINERALS;
                        }
                    }
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case EXTEND_MINERAL_SWEEPER:
                    //
                    // Deploy the sweeper.
                    //
                    robot.mineralSweeper.extend();
                    timer.set(0.3, event);
                    sm.waitForSingleEvent(event, State.DISPLACE_MINERAL);
                    break;

                case DISPLACE_MINERAL:
                    //
                    // We are supposed to be 4 inches in front of the mineral. So go forward 8 inches will displace
                    // it about 4 inches.
                    //
                    targetX = 0.0;
                    targetY = 8.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.RETRACT_MINERAL_SWEEPER);
                    break;

                case RETRACT_MINERAL_SWEEPER:
                    //
                    // Done with sweeping, retract sweeper.
                    //
                    robot.mineralSweeper.retract();
                    timer.set(0.3, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                default:
                case DONE:
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
    }

    private void setTargetY(double dist)
    {
        targetY = dist;
        distanceTravelled += dist;
    }

    public double getDistanceTravelled()
    {
        return distanceTravelled;
    }
}
