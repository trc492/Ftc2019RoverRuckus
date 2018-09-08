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

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import ftclib.FtcVuforia;

public class VuforiaVision
{
    private static final int IMAGE_WIDTH = 640;
    private static final int IMAGE_HEIGHT = 480;
    private static final int FRAME_QUEUE_CAPACITY = 2;

    private Robot robot;
    public FtcVuforia vuforia;

    public VuforiaVision(Robot robot, int cameraViewId)
    {
        final String VUFORIA_LICENSE_KEY =
                "AdCwzDH/////AAAAGeDkDS3ukU9+lIXc19LMh+cKk29caNhOl8UqmZOymRGwVwT1ZN8uaPdE3Q+zceDu9AKNsqL9qLblSFV" +
                "/x8Y3jfOZdjMFs0CQSQOEyWv3xfJsdSmevXDQDQr+4KI31HY2YSf/KB/kyxfuRMk4Pi+vWS+oLl65o7sWPiyFgzoM74ENyb" +
                "j4FgteD/2b6B+UFuwkHWKBNpp18wrpkaiFfr/FCbRFcdWP5mrjlEZM6eOj171dybw97HPeZbGihnnxOeeUv075O7P167AVq" +
                "aiPy2eRK7OCubR32KXOqQKoyF6AXp+qu2cOTApXS5dqOOseEm+HE4eMF0S2Pld3i5AWBIR+JlPXDuc9LwoH2Q8iDwUK1+4g";
        final VuforiaLocalizer.CameraDirection CAMERA_DIR = VuforiaLocalizer.CameraDirection.BACK;
        final String TRACKABLES_FILE = "RelicVuMark";

        this.robot = robot;
        vuforia = new FtcVuforia(VUFORIA_LICENSE_KEY, cameraViewId, CAMERA_DIR, TRACKABLES_FILE, 1);
        vuforia.setTargetInfo(0, "relicVuMarkTemplate");
        vuforia.configVideoSource(IMAGE_WIDTH, IMAGE_HEIGHT, FRAME_QUEUE_CAPACITY);
    }   //VuforiaVision

    public void setEnabled(boolean enabled)
    {
        vuforia.setTrackingEnabled(enabled);
    }   //setEnabled

    public RelicRecoveryVuMark getVuMark()
    {
        return RelicRecoveryVuMark.from(vuforia.getTarget(0));
    }   //getVuMark

    public VectorF getVuMarkPosition()
    {
        VectorF targetPos = null;
        VuforiaTrackable target = vuforia.getTarget(0);
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(target);

        if (vuforia.isTargetVisible(target) && vuMark != RelicRecoveryVuMark.UNKNOWN)
        {
            OpenGLMatrix pose = vuforia.getTargetPose(target);
            if (pose != null)
            {
                targetPos = pose.getTranslation();
                robot.tracer.traceInfo("TargetPos", "%s: x=%6.2f, y=%6.2f, z=%6.2f",
                                       vuMark.toString(),
                                       targetPos.get(0)/RobotInfo.MM_PER_INCH,
                                       targetPos.get(1)/RobotInfo.MM_PER_INCH,
                                       -targetPos.get(2)/RobotInfo.MM_PER_INCH);
            }
        }

        return targetPos;
    }   //getVuMarkPosition

    public Orientation getVuMarkOrientation()
    {
        Orientation targetAngle = null;
        VuforiaTrackable target = vuforia.getTarget(0);
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(target);

        if (vuforia.isTargetVisible(target) && vuMark != RelicRecoveryVuMark.UNKNOWN)
        {
            OpenGLMatrix pose = vuforia.getTargetPose(target);
            if (pose != null)
            {
                targetAngle = Orientation.getOrientation(
                        pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                robot.tracer.traceInfo("TargetRot", "%s: xRot=%6.2f, yRot=%6.2f, zRot=%6.2f",
                        vuMark.toString(),
                        targetAngle.firstAngle, targetAngle.secondAngle, targetAngle.thirdAngle);
            }
        }

        return targetAngle;
    }   //getVuMarkOrientation

}   //class VuforiaVision
