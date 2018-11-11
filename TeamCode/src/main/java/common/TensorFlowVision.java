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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.core.Rect;

import java.util.List;

import ftclib.FtcVuforia;

public class TensorFlowVision
{
    private static final boolean debugEnabled = false;

    public class TargetInfo
    {
        public String label;
        public Rect rect;
        public double angle;

        public TargetInfo(String label, Rect rect, double angle)
        {
            this.label = label;
            this.rect = rect;
            this.angle = angle;
        }   //TargetInfo

        public String toString()
        {
            return String.format("%s: Rect[%d,%d,%d,%d], angle=%.1f",
                    label, rect.x, rect.y, rect.width, rect.height, angle);
        }
    }   //class TargetInfo

    public static final int IMAGE_WIDTH = 640;      //in pixels
    public static final int IMAGE_HEIGHT = 480;     //in pixels
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private Robot robot;
    private TFObjectDetector tfod;

    public TensorFlowVision(
            Robot robot, int tfodMonitorViewId, VuforiaLocalizer.CameraDirection cameraDir)
    {
        final String VUFORIA_LICENSE_KEY =
                "ATu19Kj/////AAAAGcw4SDCVwEBSiKcUtdmQd2aOugrxo/OgeBJUt7XwMSi3e0KSZaylbsTnWp8EBxyA5o/00JFJVDY1OxJ" +
                "XLxQOpz1tbM4ex1sl1EbF25olEZ3w9xXZ1QaqMP+5T63VqTwvkgKbtM+dS+tLi8EHMvJ2viYf6WwOE776e0s3QNfl/XvONM" +
                "XS4ZtEWLNeiSEMTCdO9bdeaxnSb2RfErcmjadAThDWf6PC9HrMRHLmgfcFaZlj5JN+figOjgKhyQZeYYrcDEm0lICN5kAr2" +
                "pdfNKNOii3A80eXyTVDfPGfzTwVa4eNBY/SgmoIdBbMPb3hfZBOz7GVoVHHQWbCNbzm31p1OY+zqPPWMfzzpyiJ4mA9bLTQ";
        final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";

        this.robot = robot;
        FtcVuforia vuforia = new FtcVuforia(VUFORIA_LICENSE_KEY, -1, cameraDir);

        if (ClassFactory.getInstance().canCreateTFObjectDetector())
        {
            TFObjectDetector.Parameters tfodParameters =
                    tfodMonitorViewId == -1?
                            new TFObjectDetector.Parameters() : new TFObjectDetector.Parameters(tfodMonitorViewId);
            TFObjectDetector tfod =
                    ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia.getLocalizer());
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        }
        else
        {
            throw new UnsupportedOperationException("This device is not compatible with TensorFlow Object Detection.");
        }
    }   //TensorFlowVision

    public void setEnabled(boolean enabled)
    {
        if (enabled)
        {
            tfod.activate();
        }
        else
        {
            tfod.deactivate();
        }
    }   //setEnabled

    public void shutdown()
    {
        setEnabled(false);
        tfod.shutdown();
    }   //shutdown

    /**
     * This method gets the rectangle of the last detected target from the camera. If the camera does not have
     * any. It may mean the camera is still busy analyzing a frame or it can't find any valid target in a frame.
     * We can't tell the reason. If the camera is likely busying processing a frame, we will return the last
     * cached rectangle. Therefore, the last cached rectangle will have an expiration (i.e. cached data can be
     * stale). If the last cached data becomes stale, we will discard it and return nothing. Otherwise, we will
     * return the cached data. Of course we will return fresh data if the camera does return another rectangle,
     * in which case it will become the new cached data.
     *
     * @return rectangle of the detected target last received from the camera or last cached target if cached
     *         data has not expired. Null if no object was seen and last cached data has expired.
     */
    private Recognition getTargetObject(String label)
    {
        final String funcName = "getTargetObject";
        Recognition targetObject = null;
        //
        // getUpdatedRecognitions() will return null if no new information is available since
        // the last time that call was made.
        //
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null)
        {
            for (Recognition object: updatedRecognitions)
            {
                if (object.getLabel().equals(label))
                {
                    targetObject = object;
                    break;
                }
            }
        }

        return targetObject;
    }   //getTargetObject

    public TargetInfo getTargetInfo(String label)
    {
        final String funcName = "getTargetInfo";
        TargetInfo targetInfo = null;
        Recognition targetObject = getTargetObject(label);

        if (targetObject != null)
        {
            targetInfo = new TargetInfo(
                    targetObject.getLabel(),
                    new Rect((int)targetObject.getLeft(), (int)targetObject.getTop(),
                             (int)targetObject.getWidth(), (int)targetObject.getHeight()),
                    targetObject.estimateAngleToObject(AngleUnit.DEGREES));

            if (debugEnabled)
            {
                robot.tracer.traceInfo(funcName, "###TargetInfo###: %s", targetInfo);
            }
        }

        return targetInfo;
    }   //getTargetInfo

}   //class TensorFlowVision
