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

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import ftclib.FtcVuforia;
import trclib.TrcDbgTrace;

public class TensorFlowVision
{
    public class TargetInfo
    {
        String label;
        Rect rect;
        double angle;
        double confidence;
        int imageWidth;
        int imageHeight;
        int position;

        TargetInfo(String label, Rect rect, double angle, double confidence, int imageWidth, int imageHeight,
                   int targetPosition)
        {
            this.label = label;
            this.rect = rect;
            this.angle = angle;
            this.confidence = confidence;
            this.imageWidth = imageWidth;
            this.imageHeight = imageHeight;
            this.position = targetPosition;
        }   //TargetInfo

        public String toString()
        {
            return String.format("%s: Rect[%d,%d,%d,%d], angle=%.1f, confidence=%.3f, image(%d,%d), pos:%d",
                    label, rect.x, rect.y, rect.width, rect.height, angle, confidence, imageWidth, imageHeight,
                    position);
        }
    }   //class TargetInfo

    public static final int NUM_EXPECTED_TARGETS = 3;
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    public static final int LEFT_THRESHOLD = 425;       //210 + 215
    public static final int RIGHT_THRESHOLD = 855;      //1280 - (210 + 215)
    public static final int Y_THRESHOLD = 50;

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

    private int compareTargetY(Recognition a, Recognition b)
    {
        return (int)(a.getRight() - b.getRight());
    }   //compareTargetY

    private int compareTargetX(Recognition a, Recognition b)
    {
        return (int)(a.getTop() - b.getTop());
    }   //compareTargetX

    private ArrayList<Recognition> getTargets(String label, int numExpectedTargets)
    {
        final String funcName = "getTargets";
        ArrayList<Recognition> targets = null;
        //
        // getUpdatedRecognitions() will return null if no new information is available since
        // the last time that call was made.
        //
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null)
        {
            Recognition matchingTarget = null;
            float targetYLowThreshold;
            float targetYHighThreshold;
            //
            // Sort the list in descending Y order so that the max Y will be first.
            //
            Collections.sort(updatedRecognitions, this::compareTargetY);
            //
            // Loop through all potential targets in decreasing Y order and pick the first one that matches the label.
            //
            for (int i = 0; i < updatedRecognitions.size(); i++)
            {
                Recognition object = updatedRecognitions.get(i);
                //
                // Since the list is sorted in the order of max Y to min Y, the first one that matches the label is
                // our target.
                //
                if (matchingTarget == null && object.getLabel().equals(label))
                {
                    matchingTarget = object;
                    float targetY = object.getImageWidth() - object.getRight();
                    //
                    // Set threshold to +/- 10% of its Y coordinate.
                    //
                    targetYLowThreshold = targetY - Y_THRESHOLD;
                    targetYHighThreshold = targetY + Y_THRESHOLD;
                    //
                    // Since the list is sorted by decreasing Y, if the position of the target in the list is more
                    // than numExpectedTargets, we have picked up some false targets in the lower screen. We have
                    // to include them for the caller to do further filtering. If the position of the target is
                    // within numExpectedTargets, we will return all targets up to numExpectedTargets.
                    //
                    int numPotentialTargets = i >= numExpectedTargets? i + 1:
                                     Math.min(numExpectedTargets, updatedRecognitions.size());
                    targets = new ArrayList<>();
                    //
                    // Copy the targets that are on the same Y vicinity into a new array to be sorted by X.
                    // This will filter out the wrong targets if we only see less than numExpectedTargets on the
                    // same Y line.
                    //
                    for (int j = 0; j < numPotentialTargets; j++)
                    {
                        Recognition obj = updatedRecognitions.get(j);
                        float objY = obj.getImageWidth() - obj.getRight();
                        if (objY >= targetYLowThreshold && objY <= targetYHighThreshold)
                        {
                            targets.add(obj);
                        }
                    }

                    Collections.sort(targets, this::compareTargetX);
                }

                if (tracer != null)
                {
                    tracer.traceInfo(funcName, "[%d/%d: %s]: x=%.0f, y=%.0f, w=%.0f, h=%.0f",
                            i, updatedRecognitions.size(), object.getLabel(), object.getTop(),
                            object.getImageWidth() - object.getRight(), object.getHeight(), object.getWidth());
                    //
                    // If we found the target and dump the info of at least numExpectedTargets, we are done.
                    //
                    if (matchingTarget != null && i >= numExpectedTargets - 1)
                    {
                        break;
                    }
                }
                else if (matchingTarget != null)
                {
                    break;
                }
            }

            if (tracer != null && targets != null)
            {
                for (int i = 0; i < targets.size(); i++)
                {
                    Recognition obj = targets.get(i);
                    tracer.traceInfo(funcName, "Minerals: [%d/%d: %s]: x=%.0f, y=%.0f, w=%.0f, h=%.0f",
                            i, targets.size(), obj.getLabel(), obj.getTop(),
                            obj.getImageWidth() - obj.getRight(), obj.getHeight(), obj.getWidth());
                }
            }
        }

        return targets;
    }   //getTargets

    public TargetInfo getTargetInfo(String label, int numExpectedTargets)
    {
        final String funcName = "getTargetInfo";
        TargetInfo targetInfo = null;
        ArrayList<Recognition> targets = getTargets(label, numExpectedTargets);

        if (targets != null)
        {
            for (int i = 0; i < targets.size(); i++)
            {
                Recognition target = targets.get(i);

                if (target.getLabel().equals(label))
                {
                    Rect targetRect = new Rect(
                            (int)target.getTop(), (int)(target.getImageWidth() - target.getRight()),
                            (int)target.getHeight(), (int)target.getWidth());
                    int targetPosition;
                    //
                    // If we found the expected number of targets, the position is the index of the array.
                    // Otherwise, we will determine the position by the target's rect on the screen.
                    //
                    if (targets.size() == numExpectedTargets)
                    {
                        targetPosition = i;
                    }
                    else
                    {
                        int xCenter = targetRect.x + targetRect.width/2;

                        if (xCenter < LEFT_THRESHOLD)
                        {
                            targetPosition = 0;
                        }
                        else if (xCenter < RIGHT_THRESHOLD)
                        {
                            targetPosition = 1;
                        }
                        else
                        {
                            targetPosition = 2;
                        }
                    }
                    //
                    // Found the lowest target on the screen.
                    // Target object is presented in portrait mode but since the phone orientation is really landscape,
                    // we need to transpose the rect to landscape coordinates.
                    //
                    targetInfo = new TargetInfo(
                            target.getLabel(), targetRect, target.estimateAngleToObject(AngleUnit.DEGREES),
                            target.getConfidence(), target.getImageHeight(), target.getImageWidth(), targetPosition);

                    if (tracer != null)
                    {
                        tracer.traceInfo(funcName, "###TargetInfo###: [%d/%d] %s", i, targets.size(), targetInfo);
                    }
                }
            }
        }

        return targetInfo;
    }   //getTargetInfo

    public TargetInfo[] getAllTargetInfo(String label, int numExpectedTargets)
    {
        final String funcName = "getAllTargetInfo";
        TargetInfo[] targetsInfo = null;
        ArrayList<Recognition> targets = getTargets(label, numExpectedTargets);

        if (targets != null)
        {
            targetsInfo = new TargetInfo[targets.size()];

            for (int i = 0; i < targetsInfo.length; i++)
            {
                Recognition target = targets.get(i);

                Rect targetRect = new Rect(
                        (int)target.getTop(), (int)(target.getImageWidth() - target.getRight()),
                        (int)target.getHeight(), (int)target.getWidth());
                int xCenter = targetRect.x + targetRect.width/2;
                int targetPosition;

                if (xCenter < LEFT_THRESHOLD)
                {
                    targetPosition = 0;
                }
                else if (xCenter < RIGHT_THRESHOLD)
                {
                    targetPosition = 1;
                }
                else
                {
                    targetPosition = 2;
                }

                targetsInfo[i] = new TargetInfo(
                        target.getLabel(), targetRect, target.estimateAngleToObject(AngleUnit.DEGREES),
                        target.getConfidence(), target.getImageHeight(), target.getImageWidth(), targetPosition);


                if (tracer != null)
                {
                    tracer.traceInfo(funcName, "AllTargetsInfo: [%d/%d] %s", i, targets.size(), targetsInfo[i]);
                }
            }
        }

        return targetsInfo;
    }   //getAllTargetInfo

}   //class TensorFlowVision
