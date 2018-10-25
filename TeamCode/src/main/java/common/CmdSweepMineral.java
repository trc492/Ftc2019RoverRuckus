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

import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

public class CmdSweepMineral implements TrcRobot.RobotCommand
{
    private static final boolean debugXPid = true;
    private static final boolean debugYPid = true;
    private static final boolean debugTurnPid = true;

    enum State
    {
        START,
        SCAN_MINERALS,
        EXTEND_MINERAL_SWEEPER,
        DISPLACE_MINERAL,
        RETRACT_MINERAL_SWEEPER,
        DONE
    }

    private static final String moduleName = "CmdSweepMineral";

    private Robot robot;
    private double startingY;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;
    private double targetX = 0.0;
    private double targetY = 0.0;
    private int retries = 0;
    private double distanceYTravelled = 0.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object.
     * @param startingY specifies the starting Y distance from the middle mineral.
     */
    public CmdSweepMineral(Robot robot, double startingY)
    {
        this.robot = robot;
        this.startingY = startingY;
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START);
    }   //CmdSweepMineral

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
                    //
                    // Get to the Y position of the very first mineral which is about 25 inches back from the
                    // middle mineral. So we need to calculate the distance to go from the starting Y position.
                    // If for some reason pixy camera is not enabled, just sweep the first mineral hoping it's
                    // the right one. There is nothing to lose.
                    //
                    setTarget(0.0, -25.0 - startingY);
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    nextState = robot.pixyVision != null? State.SCAN_MINERALS: State.EXTEND_MINERAL_SWEEPER;
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case SCAN_MINERALS:
                    PixyVision.TargetInfo targetInfo =
                            robot.pixyVision.getTargetInfo(RobotInfo.PIXY_GOLD_MINERAL_SIGNATURE);
                    if (targetInfo != null)
                    {
                        //
                        // Get the sweeper right behind the gold mineral. The pixy camera is 4 inches behind the arm.
                        // We will give another 4 inches margin so a total of 8 inches.
                        //
                        setTarget(0.0, targetInfo.xDistance - 8.0);
                        nextState = State.EXTEND_MINERAL_SWEEPER;
                    }
                    else
                    {
                        //
                        // This is the distance between two minerals.
                        //
                        setTarget(0.0, 14.5);
                        retries++;
                        if (retries == 3)
                        {
                            //
                            // This is the third mineral and the pixy still failed to detect it, sweep it anyway
                            // in hope that's the right one. There's nothing to lose.
                            //
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
                    // We are supposed to be 4 inches behind the mineral. So go forward 8 inches will displace
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
    }   //cmdPeriodic

    private void setTarget(double distX, double distY)
    {
        targetX = distX;
        targetY = distY;
        distanceYTravelled += distY;
    }   //setTarget

    public double getDistanceYTravelled()
    {
        return distanceYTravelled;
    }   //getDistanceYTravelled

}   //class CmdSweepMineral
