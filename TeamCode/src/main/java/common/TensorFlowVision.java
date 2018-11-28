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
import trclib.TrcDbgTrace;

public class TensorFlowVision
{
    public class TargetInfo
    {
        public String label;
        public Rect rect;
        public double angle;
        public double confidence;
        public int imageWidth;
        public int imageHeight;

        public TargetInfo(String label, Rect rect, double angle, double confidence, int imageWidth, int imageHeight)
        {
            this.label = label;
            this.rect = rect;
            this.angle = angle;
            this.confidence = confidence;
            this.imageWidth = imageWidth;
            this.imageHeight = imageHeight;
        }   //TargetInfo

        public String toString()
        {
            return String.format("%s: Rect[%d,%d,%d,%d], angle=%.1f, confidence=%.3f, image(%d,%d)",
                    label, rect.x, rect.y, rect.width, rect.height, angle, confidence, imageWidth, imageHeight);
        }
    }   //class TargetInfo

    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private TrcDbgTrace tracer;
    private FtcVuforia vuforia;
    private TFObjectDetector tfod;

    public TensorFlowVision(
            int tfodMonitorViewId, VuforiaLocalizer.CameraDirection cameraDir, TrcDbgTrace tracer)
    {
        final String VUFORIA_LICENSE_KEY =
                "ATu19Kj/////AAAAGcw4SDCVwEBSiKcUtdmQd2aOugrxo/OgeBJUt7XwMSi3e0KSZaylbsTnWp8EBxyA5o/00JFJVDY1OxJ" +
                "XLxQOpz1tbM4ex1sl1EbF25olEZ3w9xXZ1QaqMP+5T63VqTwvkgKbtM+dS+tLi8EHMvJ2viYf6WwOE776e0s3QNfl/XvONM" +
                "XS4ZtEWLNeiSEMTCdO9bdeaxnSb2RfErcmjadAThDWf6PC9HrMRHLmgfcFaZlj5JN+figOjgKhyQZeYYrcDEm0lICN5kAr2" +
                "pdfNKNOii3A80eXyTVDfPGfzTwVa4eNBY/SgmoIdBbMPb3hfZBOz7GVoVHHQWbCNbzm31p1OY+zqPPWMfzzpyiJ4mA9bLTQ";
        final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";

        this.tracer = tracer;
        vuforia = new FtcVuforia(VUFORIA_LICENSE_KEY, -1, cameraDir);

        if (ClassFactory.getInstance().canCreateTFObjectDetector())
        {
            TFObjectDetector.Parameters tfodParameters =
                    tfodMonitorViewId == -1?
                            new TFObjectDetector.Parameters() : new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia.getLocalizer());
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

    public void setLightEnabled(boolean enabled)
    {
        vuforia.setFlashlightEnabled(enabled);
    }   //setLightEnabled

    public TargetInfo getTargetInfo(String label)
    {
        final String funcName = "getTargetInfo";
        TargetInfo targetInfo = null;
        //
        // getUpdatedRecognitions() will return null if no new information is available since
        // the last time that call was made.
        //
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null)
        {
            int potentialTargetCount = 0;
            int targetIndex = 0;
            Recognition targetObject = null;
            float targetObjectY = 0.0f;
            float landscapeHeight = 0.0f;
            //
            // Loop through all potential targets and pick the one lowest on the screen (i.e. max Y).
            //
            for (int i = 0; i < updatedRecognitions.size(); i++)
            {
                Recognition object = updatedRecognitions.get(i);

//                tracer.traceInfo(funcName, "***TargetInfo***: [%d: %s]: %.0f/%.0f/%.0f/%.0f",
//                        i, object.getLabel(), object.getTop(), object.getImageWidth() - object.getRight(),
//                        object.getHeight(), object.getWidth());
                if (object.getLabel().equals(label))
                {
                    float imageWidth = object.getImageWidth();
                    float objectY = imageWidth - object.getRight();

                    potentialTargetCount++;
                    if (targetObject == null || objectY > targetObjectY)
                    {
                        targetIndex = potentialTargetCount;
                        targetObject = object;
                        targetObjectY = objectY;
                        landscapeHeight = imageWidth;
                    }
                }
            }

            if (targetObject != null)
            {
                //
                // Found the lowest target on the screen.
                // Target object is presented in portrait mode but since the phone orientation is really landscape,
                // we need to transpose the rect to landscape coordinates.
                //
                targetInfo = new TargetInfo(
                        targetObject.getLabel(),
                        new Rect(
                                (int)targetObject.getTop(), (int)(landscapeHeight - targetObject.getRight()),
                                (int)targetObject.getHeight(), (int)targetObject.getWidth()),
                        targetObject.estimateAngleToObject(AngleUnit.DEGREES),
                        targetObject.getConfidence(),
                        targetObject.getImageHeight(), targetObject.getImageWidth());

                if (tracer != null)
                {
                    tracer.traceInfo(funcName, "###TargetInfo###: [%d/%d] %s",
                            targetIndex, potentialTargetCount, targetInfo);
                }
            }
        }

        return targetInfo;
    }   //getTargetInfo

}   //class TensorFlowVision
