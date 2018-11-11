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

public class CmdDisplaceMineral implements TrcRobot.RobotCommand
{
    private static final boolean debugXPid = true;
    private static final boolean debugYPid = true;
    private static final boolean debugTurnPid = true;

    private static final String moduleName = "CmdDisplaceMineral";

    private Robot robot;
    private boolean startAtDepot;
    private TrcEvent event;
    private TrcStateMachine<State> sm;

    private double targetX = 0.0;
    private double targetY = 0.0;
    private double mineralAngle = 0.0;
    private int retries = 0;
    private double startHeading = 0.0;
    private double startPos = 0.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object.
     */
    public CmdDisplaceMineral(Robot robot, boolean startAtDepot)
    {
        this.robot = robot;
        this.startAtDepot = startAtDepot;
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.SCAN_MINERAL);
    }   //CmdSweepMineral

    enum State
    {
        SCAN_MINERAL,
        TURN_TO_MINERAL,
        DISPLACE_MINERAL,
        BACK_TO_START_POSITION,
        TURN_TO_DEPOT,
        PLOW_TO_DEPOT,
        DONE
    }

    /**
     * This method performs the task of scanning for gold mineral and displacing it. It assumes it is starting in
     * front of the middle mineral and will use vision to determine which mineral is gold. It will then turn to
     * the gold mineral and drive forward to displace it. It will also keep track of the robot ending position and
     * provide methods for the client to retrieve this position.
     *
     * @param elapsedTime specifies the elapsed time of the period in seconds.
     * @return true if the task is completed, false otherwise.
     */
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
                case SCAN_MINERAL:
                    //
                    // Use vision to scan for gold mineral. If there is no vision, assume the middle one is gold.
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
                            int leftThird = TensorFlowVision.IMAGE_WIDTH/3;
                            int rightThird = leftThird*2;
                            int mineralX = targetInfo.rect.x + targetInfo.rect.width/2;

                            if (mineralX < leftThird)
                            {
                                //
                                // Gold is the left mineral.
                                //
                                mineralAngle = -45.0;
                            }
                            else if (mineralX < rightThird)
                            {
                                //
                                // Gold is the right mineral.
                                //
                                mineralAngle = 0.0;
                            }
                            else
                            {
                                //
                                // Gold is the right mineral.
                                //
                                mineralAngle = 45.0;
                            }

                            robot.tracer.traceInfo(moduleName, "%s[%d]: found (%s).",
                                    state, retries, targetInfo);
                            sm.setState(State.TURN_TO_MINERAL);
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
                                //
                                // We tried but can't find it, so assume the middle is gold and hope for the best.
                                //
                                mineralAngle = 0.0;
                                sm.setState(State.TURN_TO_MINERAL);
                            }
                        }
                    }
                    else
                    {
                        //
                        // Vision is not enabled so assume the middle is gold and hope for the best.
                        //
                        mineralAngle = 0.0;
                        sm.setState(State.TURN_TO_MINERAL);
                    }
                    //
                    // Intentionally falling through.
                    //
                case TURN_TO_MINERAL:
                    startHeading = robot.targetHeading;
                    targetX = 0.0;
                    targetY = 0.0;
                    robot.targetHeading = mineralAngle;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DISPLACE_MINERAL);
                    break;

                case DISPLACE_MINERAL:
                    startPos = robot.driveBase.getYPosition();
                    targetX = 0.0;
                    targetY = mineralAngle == 0.0? 24.0: 36.0;
                    nextState = startAtDepot && mineralAngle == 0.0? State.PLOW_TO_DEPOT: State.BACK_TO_START_POSITION;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case BACK_TO_START_POSITION:
                    targetX = 0.0;
                    targetY = startPos - robot.driveBase.getYPosition();
                    nextState = startAtDepot? State.TURN_TO_DEPOT: State.DONE;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case TURN_TO_DEPOT:
                    targetX = targetY = 0.0;
                    robot.targetHeading = startHeading;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.PLOW_TO_DEPOT);
                    break;

                case PLOW_TO_DEPOT:
                    targetX = 0.0;
                    targetY = mineralAngle == 0.0? 24.0: 36.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                default:
                case DONE:
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

            if (debugXPid && robot.encoderXPidCtrl != null)
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

    public double getMineralAngle()
    {
        return mineralAngle;
    }   //getMineralAngle

}   //class CmdDisplaceMineral
