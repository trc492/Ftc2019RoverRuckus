/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
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

import android.speech.tts.TextToSpeech;
import android.widget.TextView;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import ftclib.FtcAnalogInput;
import ftclib.FtcAndroidTone;
import ftclib.FtcBNO055Imu;
import ftclib.FtcColorSensor;
import ftclib.FtcDcMotor;
import ftclib.FtcDigitalOutput;
import ftclib.FtcMRRangeSensor;
import ftclib.FtcMenu;
import ftclib.FtcOpMode;
import ftclib.FtcRobotBattery;
import hallib.HalDashboard;
import trclib.TrcAnalogTrigger;
import trclib.TrcDbgTrace;
import trclib.TrcDriveBase;
import trclib.TrcFilter;
import trclib.TrcGyro;
import trclib.TrcMaxbotixSonarArray;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcRobot;
import trclib.TrcSpuriousFilter;

public class Robot implements
        TrcPidController.PidInput, FtcMenu.MenuButtons, TrcAnalogTrigger.TriggerHandler, TrcPidDrive.StuckWheelHandler
{
    static final boolean USE_SPEECH = true;
    static final boolean USE_VUFORIA = true;
    static final boolean USE_GRIPVISION = false;
    static final boolean USE_JEWEL_COLOR_SENSOR = true;
    static final boolean USE_CRYPTO_COLOR_SENSOR = false;
    static final boolean USE_ANALOG_TRIGGERS = false;
    static final boolean USE_DOG_LEASH = true;
    static final boolean USE_MRRANGE_SENSOR = true;
    static final boolean USE_RANGE_DRIVE = true;
    static final boolean USE_MAXBOTIX_SONAR_SENSOR = false;
    static final boolean USE_SONAR_DRIVE = false;

    static final int LEFT_SONAR_INDEX = 0;
    static final int FRONT_SONAR_INDEX = 1;
    static final int RIGHT_SONAR_INDEX = 2;

    private static final String moduleName = "Robot";


    enum ObjectColor
    {
        NO,
        RED,
        BLUE
    }

    //
    // Global objects.
    //
    FtcOpMode opMode;
    HalDashboard dashboard;
    TrcDbgTrace tracer;
    FtcRobotBattery battery = null;
    FtcAndroidTone androidTone;
    TextToSpeech textToSpeech = null;
    //
    // Sensors.
    //
    FtcBNO055Imu imu = null;
    TrcGyro gyro = null;
    double targetHeading = 0.0;

    static final double[] colorTriggerPoints = {
            RobotInfo.RED1_LOW_THRESHOLD, RobotInfo.RED1_HIGH_THRESHOLD,
            RobotInfo.BLUE_LOW_THRESHOLD, RobotInfo.BLUE_HIGH_THRESHOLD,
            RobotInfo.RED2_LOW_THRESHOLD, RobotInfo.RED2_HIGH_THRESHOLD};

    FtcColorSensor jewelColorSensor = null;
    TrcAnalogTrigger<FtcColorSensor.DataType> jewelColorTrigger = null;

    FtcColorSensor cryptoColorSensor = null;
    TrcAnalogTrigger<FtcColorSensor.DataType> cryptoColorTrigger = null;
    int redCryptoBarCount = 0;
    int blueCryptoBarCount = 0;

    FtcMRRangeSensor leftRangeSensor = null;
    FtcMRRangeSensor rightRangeSensor = null;
    FtcAnalogInput leftSonar = null;
    FtcAnalogInput frontSonar = null;
    FtcAnalogInput rightSonar = null;
    FtcAnalogInput[] sonarSensors = new FtcAnalogInput[3];
    FtcDigitalOutput sonarRX = null;
    TrcMaxbotixSonarArray sonarArray = null;
    boolean useRightSensorForX = false;

    //
    // Vision subsystems.
    //
    VuforiaVision vuforiaVision = null;
    RelicRecoveryVuMark prevVuMark = null;
    GripVision gripVision = null;
    //
    // DriveBase subsystem.
    //
    FtcDcMotor leftFrontWheel = null;
    FtcDcMotor rightFrontWheel = null;
    FtcDcMotor leftRearWheel = null;
    FtcDcMotor rightRearWheel = null;
    TrcDriveBase driveBase = null;

    TrcPidController encoderXPidCtrl = null;
    TrcPidController encoderYPidCtrl = null;
    TrcPidController gyroPidCtrl = null;
    TrcPidDrive pidDrive = null;

    TrcPidController visionPidCtrl = null;
    TrcPidDrive visionDrive = null;

    TrcPidController rangeXPidCtrl = null;
    TrcPidDrive rangeXPidDrive = null;
    TrcSpuriousFilter spuriousLeftRangeFilter = null;
    TrcSpuriousFilter spuriousRightRangeFilter = null;

    TrcPidController sonarXPidCtrl = null;
    TrcPidController sonarYPidCtrl = null;
    TrcPidDrive sonarXPidDrive = null;
    TrcPidDrive sonarYPidDrive = null;
    TrcSpuriousFilter spuriousLeftSonarFilter = null;
    TrcSpuriousFilter spuriousFrontSonarFilter = null;
    TrcSpuriousFilter spuriousRightSonarFilter = null;
    Double prevXDistance = null;
    double xOffset = 0.0;

    //
    // Other subsystems.
    //
    GlyphElevator glyphElevator = null;
    GlyphGrabber glyphGrabber = null;
    JewelArm jewelArm = null;
    RelicArm relicArm = null;

    public Robot(TrcRobot.RunMode runMode)
    {
        //
        // Initialize global objects.
        //
        opMode = FtcOpMode.getInstance();
        opMode.hardwareMap.logDevices();
        dashboard = HalDashboard.getInstance();
        tracer = FtcOpMode.getGlobalTracer();
        dashboard.setTextView(
                (TextView)((FtcRobotControllerActivity)opMode.hardwareMap.appContext).findViewById(R.id.textOpMode));
        battery = new FtcRobotBattery();
        androidTone = new FtcAndroidTone("AndroidTone");
        if (USE_SPEECH)
        {
            textToSpeech = FtcOpMode.getInstance().getTextToSpeech();
            textToSpeech.speak("Init starting", TextToSpeech.QUEUE_FLUSH, null);
        }
        //
        // Initialize sensors.
        //
        imu = new FtcBNO055Imu("imu2");
        gyro = imu.gyro;

        if (USE_JEWEL_COLOR_SENSOR)
        {
            jewelColorSensor = new FtcColorSensor("jewelColorRangeSensor");
            if (USE_ANALOG_TRIGGERS)
            {
                jewelColorTrigger = new TrcAnalogTrigger(
                        "jewelColorTrigger", jewelColorSensor, 0, FtcColorSensor.DataType.HUE,
                        colorTriggerPoints, this);
            }
        }

        if (USE_CRYPTO_COLOR_SENSOR)
        {
            cryptoColorSensor = new FtcColorSensor("cryptoColorRangeSensor");
            if (USE_ANALOG_TRIGGERS)
            {
                cryptoColorTrigger = new TrcAnalogTrigger(
                        "cryptoColorTrigger", cryptoColorSensor, 0, FtcColorSensor.DataType.HUE,
                        colorTriggerPoints, this);
            }
        }

        if (USE_MRRANGE_SENSOR)
        {
            spuriousLeftRangeFilter = new TrcSpuriousFilter(
                    "leftRangeFilter", RobotInfo.RANGE_ERROR_THRESHOLD, tracer);
            leftRangeSensor = new FtcMRRangeSensor(
                    "leftRangeSensor", new TrcFilter[] {spuriousLeftRangeFilter});

            spuriousRightRangeFilter = new TrcSpuriousFilter(
                    "rightRangeFilter", RobotInfo.RANGE_ERROR_THRESHOLD, tracer);
            rightRangeSensor = new FtcMRRangeSensor(
                    "rightRangeSensor", new TrcFilter[] {spuriousRightRangeFilter});
        }

        if (USE_MAXBOTIX_SONAR_SENSOR)
        {
            spuriousLeftSonarFilter = new TrcSpuriousFilter(
                    "leftSonarFilter", RobotInfo.SONAR_ERROR_THRESHOLD, tracer);
            leftSonar = new FtcAnalogInput("leftSonar", new TrcFilter[] {spuriousLeftSonarFilter});
            leftSonar.setScale(RobotInfo.SONAR_INCHES_PER_VOLT);

            spuriousFrontSonarFilter = new TrcSpuriousFilter(
                    "frontSonarFilter", RobotInfo.SONAR_ERROR_THRESHOLD, tracer);
            frontSonar = new FtcAnalogInput("frontSonar", new TrcFilter[] {spuriousFrontSonarFilter});
            frontSonar.setScale(RobotInfo.SONAR_INCHES_PER_VOLT);

            spuriousRightSonarFilter = new TrcSpuriousFilter(
                    "rightSonarFilter", RobotInfo.SONAR_ERROR_THRESHOLD, tracer);
            rightSonar = new FtcAnalogInput("rightSonar", new TrcFilter[] {spuriousRightSonarFilter});
            rightSonar.setScale(RobotInfo.SONAR_INCHES_PER_VOLT);

            sonarSensors[LEFT_SONAR_INDEX] = leftSonar;
            sonarSensors[FRONT_SONAR_INDEX] = frontSonar;
            sonarSensors[RIGHT_SONAR_INDEX] = rightSonar;
            sonarRX = new FtcDigitalOutput("sonarRX");
            sonarArray = new TrcMaxbotixSonarArray("sonarArray", sonarSensors, sonarRX);
        }

        //
        // Initialize vision subsystems.
        //
        if (USE_VUFORIA)
        {
            int cameraViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                    "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
            vuforiaVision = new VuforiaVision(this, -1);//cameraViewId);

            if (USE_GRIPVISION)
            {
                gripVision = new GripVision("gripVision", vuforiaVision.vuforia);
                gripVision.setVideoOutEnabled(false);
            }
        }
        //
        // Initialize DriveBase.
        //
        leftFrontWheel = new FtcDcMotor("leftFrontWheel");
        rightFrontWheel = new FtcDcMotor("rightFrontWheel");
        leftRearWheel = new FtcDcMotor("leftRearWheel");
        rightRearWheel = new FtcDcMotor("rightRearWheel");

        leftFrontWheel.motor.setMode(RobotInfo.DRIVE_MOTOR_MODE);
        rightFrontWheel.motor.setMode(RobotInfo.DRIVE_MOTOR_MODE);
        leftRearWheel.motor.setMode(RobotInfo.DRIVE_MOTOR_MODE);
        rightRearWheel.motor.setMode(RobotInfo.DRIVE_MOTOR_MODE);

        leftFrontWheel.setInverted(false);
        leftRearWheel.setInverted(false);
        rightFrontWheel.setInverted(true);
        rightRearWheel.setInverted(true);

        leftFrontWheel.setBrakeModeEnabled(true);
        leftRearWheel.setBrakeModeEnabled(true);
        rightFrontWheel.setBrakeModeEnabled(true);
        rightRearWheel.setBrakeModeEnabled(true);

        driveBase = new TrcDriveBase(leftFrontWheel, leftRearWheel, rightFrontWheel, rightRearWheel, gyro);
        driveBase.setXPositionScale(RobotInfo.ENCODER_X_INCHES_PER_COUNT);
        driveBase.setYPositionScale(RobotInfo.ENCODER_Y_INCHES_PER_COUNT);
        //
        // Initialize PID drive.
        //
        encoderXPidCtrl = new TrcPidController(
                "encoderXPidCtrl",
                new TrcPidController.PidCoefficients(
                        RobotInfo.ENCODER_X_KP, RobotInfo.ENCODER_X_KI, RobotInfo.ENCODER_X_KD),
                RobotInfo.ENCODER_X_TOLERANCE, this);
        encoderYPidCtrl = new TrcPidController(
                "encoderYPidCtrl",
                new TrcPidController.PidCoefficients(
                        RobotInfo.ENCODER_Y_KP, RobotInfo.ENCODER_Y_KI, RobotInfo.ENCODER_Y_KD),
                RobotInfo.ENCODER_Y_TOLERANCE, this);
        gyroPidCtrl = new TrcPidController(
                "gyroPidCtrl",
                new TrcPidController.PidCoefficients(
                        RobotInfo.GYRO_KP, RobotInfo.GYRO_KI, RobotInfo.GYRO_KD),
                RobotInfo.GYRO_TOLERANCE, this);
        gyroPidCtrl.setAbsoluteSetPoint(true);
        gyroPidCtrl.setOutputRange(-RobotInfo.TURN_POWER_LIMIT, RobotInfo.TURN_POWER_LIMIT);

        pidDrive = new TrcPidDrive("pidDrive", driveBase, encoderXPidCtrl, encoderYPidCtrl, gyroPidCtrl);
        pidDrive.setStallTimeout(RobotInfo.PIDDRIVE_STALL_TIMEOUT);
//        pidDrive.setStuckWheelHandler(this, RobotInfo.PIDDRIVE_STALL_TIMEOUT);
        pidDrive.setBeep(androidTone);

        visionPidCtrl = new TrcPidController(
                "visionPidCtrl",
                new TrcPidController.PidCoefficients(
                        RobotInfo.VISION_KP, RobotInfo.VISION_KI, RobotInfo.VISION_KD),
                RobotInfo.VISION_TOLERANCE, this);
        visionPidCtrl.setAbsoluteSetPoint(true);

        visionDrive = new TrcPidDrive("visionDrive", driveBase, null, visionPidCtrl, gyroPidCtrl);
        visionDrive.setStallTimeout(RobotInfo.PIDDRIVE_STALL_TIMEOUT);
        visionDrive.setBeep(androidTone);

        if (USE_MRRANGE_SENSOR && USE_RANGE_DRIVE)
        {
            rangeXPidCtrl = new TrcPidController(
                    "rangeXPidCtrl",
                    new TrcPidController.PidCoefficients(
                            RobotInfo.RANGE_X_KP, RobotInfo.RANGE_X_KI, RobotInfo.RANGE_X_KD),
                    RobotInfo.RANGE_X_TOLERANCE, this);
            rangeXPidCtrl.setAbsoluteSetPoint(true);
            rangeXPidCtrl.setInverted(true);

            rangeXPidDrive = new TrcPidDrive(
                    "rangeXDrive", driveBase, rangeXPidCtrl, encoderYPidCtrl, gyroPidCtrl);
            rangeXPidDrive.setStallTimeout(RobotInfo.PIDDRIVE_STALL_TIMEOUT);
            rangeXPidDrive.setBeep(androidTone);
        }

        if (USE_MAXBOTIX_SONAR_SENSOR && USE_SONAR_DRIVE)
        {
            sonarXPidCtrl = new TrcPidController(
                    "sonarXPidCtrl",
                    new TrcPidController.PidCoefficients(
                            RobotInfo.SONAR_X_KP, RobotInfo.SONAR_X_KI, RobotInfo.SONAR_X_KD),
                    RobotInfo.SONAR_X_TOLERANCE, this);
            sonarXPidCtrl.setAbsoluteSetPoint(true);
            sonarXPidCtrl.setInverted(true);

            sonarYPidCtrl = new TrcPidController(
                    "sonarYPidCtrl",
                    new TrcPidController.PidCoefficients(
                            RobotInfo.SONAR_Y_KP, RobotInfo.SONAR_Y_KI, RobotInfo.SONAR_Y_KD),
                    RobotInfo.SONAR_Y_TOLERANCE, this);
            sonarYPidCtrl.setAbsoluteSetPoint(true);
            sonarYPidCtrl.setInverted(true);

            sonarXPidDrive = new TrcPidDrive(
                    "sonarXPidDrive", driveBase, sonarXPidCtrl, encoderYPidCtrl, gyroPidCtrl);
            sonarXPidDrive.setStallTimeout(RobotInfo.PIDDRIVE_STALL_TIMEOUT);
            sonarXPidDrive.setBeep(androidTone);

            sonarYPidDrive = new TrcPidDrive(
                    "sonarYPidDrive", driveBase, encoderXPidCtrl, sonarYPidCtrl, gyroPidCtrl);
            sonarYPidDrive.setStallTimeout(RobotInfo.PIDDRIVE_STALL_TIMEOUT);
            sonarYPidDrive.setBeep(androidTone);
        }

        //
        // Initialize other subsystems.
        //

        glyphElevator = new GlyphElevator();
        if (runMode != TrcRobot.RunMode.TELEOP_MODE)
        {
            glyphElevator.zeroCalibrate();
        }

        glyphGrabber = new GlyphGrabber("glyphGrabber");
        glyphGrabber.setPosition(RobotInfo.GLYPH_GRABBER_START);

        jewelArm = new JewelArm("jewelArm");
        jewelArm.setExtended(false);
        jewelArm.setSweepPosition(RobotInfo.JEWEL_ARM_NEUTRAL);

        relicArm = new RelicArm();
        if (runMode != TrcRobot.RunMode.TELEOP_MODE)
        {
            relicArm.elbow.zeroCalibrate(RobotInfo.RELIC_ELBOW_CAL_POWER);
        }
        relicArm.grabber.setPosition(RobotInfo.RELIC_GRABBER_CLOSE);

        //
        // Tell the driver initialization is complete.
        //
        if (textToSpeech != null)
        {
            textToSpeech.speak("Init complete!", TextToSpeech.QUEUE_ADD, null);
        }
    }   //Robot

    void startMode(TrcRobot.RunMode runMode)
    {
        //
        // Since the IMU gyro is giving us cardinal heading, we need to enable its cardinal to cartesian converter.
        //
        gyro.resetZIntegrator();
        gyro.setEnabled(true);

        if (vuforiaVision != null)
        {
            vuforiaVision.setEnabled(true);
        }

        if (gripVision != null)
        {
            gripVision.setEnabled(true);
        }

        //
        // Reset all X, Y and heading values.
        //
        driveBase.resetPosition();
        targetHeading = 0.0;
        glyphGrabber.setPosition(RobotInfo.GLYPH_GRABBER_OPEN);
    }   //startMode

    void stopMode(TrcRobot.RunMode runMode)
    {
        if (gripVision != null)
        {
            gripVision.setEnabled(false);
        }

        if (vuforiaVision != null)
        {
            vuforiaVision.setEnabled(false);
        }
        //
        // Disable the gyro integrator.
        //
        gyro.setEnabled(false);

        if (textToSpeech != null)
        {
            textToSpeech.stop();
            textToSpeech.shutdown();
        }
    }   //stopMode

    void traceStateInfo(double elapsedTime, String stateName, double xDistance, double yDistance, double heading)
    {
        tracer.traceInfo(
                moduleName,
                "[%5.3f] >>>>> %s: xPos=%6.2f/%6.2f,yPos=%6.2f/%6.2f,heading=%6.1f/%6.1f,volt=%5.2fV(%5.2fV)",
                elapsedTime, stateName,
                driveBase.getXPosition(), xDistance, driveBase.getYPosition(), yDistance,
                driveBase.getHeading(), heading,
                battery.getVoltage(), battery.getLowestVoltage());
    }   //traceStateInfo

    ObjectColor getObjectColor(FtcColorSensor sensor)
    {
        ObjectColor color = ObjectColor.NO;

        if (sensor != null)
        {
            double hue = sensor.getProcessedData(0, FtcColorSensor.DataType.HUE).value;
            double sat = sensor.getProcessedData(0, FtcColorSensor.DataType.SATURATION).value;
            double value = sensor.getProcessedData(0, FtcColorSensor.DataType.VALUE).value;

            if (sat > 0.0 && value > 0.0)
            {
                if (hue >= RobotInfo.RED1_LOW_THRESHOLD && hue <= RobotInfo.RED1_HIGH_THRESHOLD ||
                    hue >= RobotInfo.RED2_LOW_THRESHOLD && hue <= RobotInfo.RED2_HIGH_THRESHOLD)
                {
                    color = ObjectColor.RED;
                }
                else if (hue >= RobotInfo.BLUE_LOW_THRESHOLD && hue <= RobotInfo.BLUE_HIGH_THRESHOLD)
                {
                    color = ObjectColor.BLUE;
                }
            }
        }

        return color;
    }   //getObjectColor

    double getObjectHsvHue(FtcColorSensor sensor)
    {
        double value = 0.0;

        if (sensor != null)
        {
            value = sensor.getProcessedData(0, FtcColorSensor.DataType.HUE).value;
        }

        return value;
    }   //getObjectHsvHue

    double getObjectHsvSaturation(FtcColorSensor sensor)
    {
        double value = 0.0;

        if (sensor != null)
        {
            value = sensor.getProcessedData(0, FtcColorSensor.DataType.SATURATION).value;
        }

        return value;
    }   //getObjectHsvSaturation

    double getObjectHsvValue(FtcColorSensor sensor)
    {
        double value = 0.0;

        if (sensor != null)
        {
            value = sensor.getProcessedData(0, FtcColorSensor.DataType.VALUE).value;
        }

        return value;
    }   //getObjectHsvValue

    public double getRangeDistance(FtcMRRangeSensor rangeSensor)
    {
        return rangeSensor.getProcessedData(0, FtcMRRangeSensor.DataType.DISTANCE_INCH).value;
    }   //getRangeDistance

    //
    // Implements TrcPidController.PidInput
    //

    @Override
    public double getInput(TrcPidController pidCtrl)
    {
        final String funcName = "getInput";
        double input = 0.0;

        if (pidCtrl == encoderXPidCtrl)
        {
            input = driveBase.getXPosition();
            if (prevXDistance != null && Math.abs(input - prevXDistance) >= 5.0)
            {
                tracer.traceWarn(funcName, "Detected invalid X position (prev=%.3f, curr=%.3f).",
                        prevXDistance, input);
                xOffset += input - prevXDistance;
            }
            prevXDistance = input;
            input -= xOffset;
        }
        else if (pidCtrl == encoderYPidCtrl)
        {
            input = driveBase.getYPosition();
        }
        else if (pidCtrl == gyroPidCtrl)
        {
            input = driveBase.getHeading();
        }
        else if (pidCtrl == visionPidCtrl)
        {
            RelicRecoveryVuMark vuMark = vuforiaVision.getVuMark();

            if (vuMark != RelicRecoveryVuMark.UNKNOWN)
            {
                input = vuforiaVision.getVuMarkPosition().get(0)/RobotInfo.MM_PER_INCH;
            }

            if (textToSpeech != null && vuMark != prevVuMark)
            {
                String sentence = null;

                if (vuMark != RelicRecoveryVuMark.UNKNOWN)
                {
                    sentence = String.format("%s is %s.", vuMark.toString(), "in view");
                }
                else if (prevVuMark != null)
                {
                    sentence = String.format("%s is %s.", prevVuMark.toString(), "out of view");
                }

                if (sentence != null)
                {
                    textToSpeech.speak(sentence, TextToSpeech.QUEUE_FLUSH, null);
                }
            }
        }
        else if (pidCtrl == rangeXPidCtrl)
        {
            input = getRangeDistance(useRightSensorForX? rightRangeSensor: leftRangeSensor);
        }
        else if (pidCtrl == sonarXPidCtrl)
        {
            if (useRightSensorForX)
            {
                input = sonarArray.getDistance(RIGHT_SONAR_INDEX).value;
            }
            else
            {
                input = sonarArray.getDistance(LEFT_SONAR_INDEX).value;
            }
        }
        else if (pidCtrl == sonarYPidCtrl)
        {
            //
            // Read front sonar value.
            //
            input = sonarArray.getDistance(FRONT_SONAR_INDEX).value;
        }

        return input;
    }   //getInput

    //
    // Implements TrcAnalogTrigger.TriggerHandler interface.
    //

    @Override
    public void triggerEvent(TrcAnalogTrigger<?> analogTrigger, int zoneIndex, double zoneValue)
    {
        FtcColorSensor colorSensor = analogTrigger == jewelColorTrigger? jewelColorSensor: cryptoColorSensor;
        ObjectColor color = getObjectColor(colorSensor);

        if (analogTrigger == cryptoColorTrigger)
        {
            if (color == ObjectColor.RED)
            {
                redCryptoBarCount++;
                if (textToSpeech != null)
                {
                    textToSpeech.speak(
                            String.format("%d red crypto.", redCryptoBarCount), TextToSpeech.QUEUE_FLUSH, null);
                }
            }
            else if (color == ObjectColor.BLUE)
            {
                blueCryptoBarCount++;
                if (textToSpeech != null)
                {
                    textToSpeech.speak(
                            String.format("%d blue crypto.", blueCryptoBarCount), TextToSpeech.QUEUE_FLUSH, null);
                }
            }
        }
        else if (textToSpeech != null)
        {
            textToSpeech.speak(
                    String.format("%s jewel found.", color.toString()), TextToSpeech.QUEUE_FLUSH, null);
        }
    }   // triggerEvent

    //
    // Implements TrcPidDrive.StuckWheelHandler interface.
    //
    @Override
    public void stuckWheel(TrcPidDrive pidDrive, TrcDriveBase.MotorType motorType)
    {
        if (textToSpeech != null)
        {
            textToSpeech.speak(String.format("%s wheel is stuck!", motorType), TextToSpeech.QUEUE_ADD, null);
        }
    }   //stuckWheel

    //
    // Implements FtcMenu.MenuButtons interface.
    //

    @Override
    public boolean isMenuUpButton()
    {
        return opMode.gamepad1.dpad_up;
    }   //isMenuUpButton

    @Override
    public boolean isMenuDownButton()
    {
        return opMode.gamepad1.dpad_down;
    }   //isMenuDownButton

    @Override
    public boolean isMenuEnterButton()
    {
        return opMode.gamepad1.a;
    }   //isMenuEnterButton

    @Override
    public boolean isMenuBackButton()
    {
        return opMode.gamepad1.dpad_left;
    }   //isMenuBackButton

}   //class Robot
