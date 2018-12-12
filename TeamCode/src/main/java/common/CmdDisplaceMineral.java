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
    private int team;
    private boolean startAtDepot;
    private double sideMineralAngle;
    private double unhookDisplacement;
    private TrcEvent event;
    private TrcStateMachine<State> sm;

    private double targetX = 0.0;
    private double targetY = 0.0;
    private double mineralAngle = 0.0;
    private double startPos = 0.0;

    public CmdDisplaceMineral(
            Robot robot, int team, boolean startAtDepot, double sideMineralAngle, double unhookDisplacement)
    {
        this.robot = robot;
        this.team = team;
        this.startAtDepot = startAtDepot;
        this.sideMineralAngle = sideMineralAngle;
        this.unhookDisplacement = unhookDisplacement;
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.SCAN_MINERAL);
    }   //CmdDisplaceMineral

    enum State
    {
        SCAN_MINERAL,
        TURN_TO_MINERAL,
        DISPLACE_MINERAL,
        BACK_TO_START_POSITION,
        TURN_TO_DEPOT,
        DRIVE_TO_DEPOT,
        DONE
    }   //enum State

    /**
     * This method performs the task of scanning for gold mineral and displacing it. It assumes it is starting in
     * front of the middle mineral and will use vision to determine which mineral is gold. It will then turn to
     * the gold mineral and drive forward to displace it. It also keeps track of the mineral angle and provide
     * a method to access it in case the caller may want the info.
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
                    // If vision is enabled, it would have scanned for the gold mineral at InitPeriodic time.
                    // So we just need to check for the result here. If none found, assume the middle one is gold.
                    //
                    if (robot.targetInfo != null)
                    {
                        //
                        // Found gold mineral.
                        //
                        String sentence;

                        if (robot.targetInfo.position == 0)
                        {
                            //
                            // Gold is the left mineral.
                            //
                            mineralAngle = -sideMineralAngle;
                            sentence = "Gold mineral is on the left.";
                        }
                        else if (robot.targetInfo.position == 1)
                        {
                            //
                            // Gold is the right mineral.
                            //
                            mineralAngle = 0.0;
                            sentence = "Gold mineral is in the middle.";
                        }
                        else
                        {
                            //
                            // Gold is the right mineral.
                            //
                            mineralAngle = sideMineralAngle;
                            sentence = "Gold mineral is on the right.";
                        }

                        robot.speak(sentence);
                        robot.globalTracer.traceInfo(moduleName, "%s: %s (%s).",
                                state, sentence, robot.targetInfo);
                        sm.setState(State.TURN_TO_MINERAL);
                    }
                    else
                    {
                        //
                        // Vision did not find gold mineral so assume the middle is gold and hope for the best.
                        //
                        mineralAngle = 0.0;
                        sm.setState(State.TURN_TO_MINERAL);
                    }
                    //
                    // Intentionally falling through.
                    //
                case TURN_TO_MINERAL:
                    //
                    // Turn to the gold mineral (hopefully).
                    //
                    targetX = 0.0;
                    targetY = 0.0;
                    robot.targetHeading = mineralAngle;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event, 2.0);
                    sm.waitForSingleEvent(event, State.DISPLACE_MINERAL);
                    break;

                case DISPLACE_MINERAL:
                    //
                    // Drive forward to displace the mineral.
                    //
                    startPos = robot.driveBase.getYPosition();
                    targetX = mineralAngle > 0.0? 0.0: -unhookDisplacement;
                    if (startAtDepot)
                    {
                        //
                        // We are starting on the depot side. It means we will end inside the depot.
                        //

                        if (team == 6541)
                        {
                            targetY = mineralAngle == 0.0? 58.0: 40.0;
                        }
                        else
                        {
                            targetY = mineralAngle == 0.0? 56.0: 44.0; // prev side: 48in
                        }

                        nextState = mineralAngle == 0.0? State.DONE: State.TURN_TO_DEPOT;
                    }
                    else
                    {
                        //
                        // We are starting on the crater side. It means we will end at our starting position.
                        //
                        if (team == 3543)
                        {
                            targetY = mineralAngle == 0.0? 30.0: mineralAngle > 0.0? 28.0: 30.0;
                        }
                        else
                        {
                            targetY = mineralAngle == 0.0? 30.0: mineralAngle > 0.0? 35.0: 35.0;
                        }
                        nextState = State.BACK_TO_START_POSITION;
                    }
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case BACK_TO_START_POSITION:
                    //
                    // We are starting on the crater side so we will go back to our starting position.
                    // By subtracting our current Y position from the start position recorded, we cancel
                    // out any PID drive error that may have been accumulated.
                    //
                    double offset = mineralAngle < 0.0? 20.0: 15.0;
                    targetX = 0.0;
                    targetY = startPos + offset - robot.driveBase.getYPosition();
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case TURN_TO_DEPOT:
                    //
                    // We are starting on the depot side and have displaced either the left or the right mineral.
                    // We need to turn towards the depot.
                    //
                    targetX = targetY = 0.0;
                    if (team == 3543)
                    {
                        robot.targetHeading += mineralAngle == -sideMineralAngle ? 80.0 : -80.0; // prev: 90.0 : -90.0
                    }
                    else
                    {
                        robot.targetHeading += mineralAngle == -sideMineralAngle? 75.0: -75.0; // prev: 90.0 : -90.0
                    }
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_DEPOT);
                    break;

                case DRIVE_TO_DEPOT:
                    //
                    // Drive forward to the depot and we are done.
                    //
                    targetX = 0.0;
                    targetY = team == 3543? 44.0: 42.0;
                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                default:
                case DONE:
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

            if (debugXPid && robot.encoderXPidCtrl != null)
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

    /**
     * This method returns the mineral angle it has determined probably by vision.
     *
     * @return mineral angle from the starting position.
     */
    @SuppressWarnings("unused")
    public double getMineralAngle()
    {
        return mineralAngle;
    }   //getMineralAngle

}   //class CmdDisplaceMineral
