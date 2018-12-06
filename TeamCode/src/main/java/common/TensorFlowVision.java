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

import java.util.Arrays;
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

        TargetInfo(String label, Rect rect, double angle, double confidence, int imageWidth, int imageHeight)
        {
            this.label = label;
            this.rect = rect;
            this.angle = angle;
            this.confidence = confidence;
            this.imageWidth = imageWidth;
            this.imageHeight = imageHeight;
            this.position = -1;
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
    public static final int LEFT_THRESHOLD = 1280/3;
    public static final int RIGHT_THRESHOLD = 1280*2/3;

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
        return (int)(b.getTop() - a.getTop());
    }   //compareTargetX

    private Recognition[] getTargets(String label, int numExpectedTargets)
    {
        final String funcName = "getTargets";
        Recognition[] targets = null;
        //
        // getUpdatedRecognitions() will return null if no new information is available since
        // the last time that call was made.
        //
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null)
        {
            Recognition matchingTarget = null;
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
                    //
                    // Since the list is sorted by decreasing Y, if the position of the target in the list is more
                    // than numExpectedTargets, we have picked up some false targets in the lower screen. We have
                    // to include them for the caller to do further filtering. If the position of the target is
                    // within numExpectedTargets, we will return all targets up to numExpectedTargets.
                    //
                    int numTargets = i >= numExpectedTargets? i + 1:
                                     Math.min(numExpectedTargets, updatedRecognitions.size());
                    targets = new Recognition[numTargets];
                    //
                    // Copy the targets into a new array to be sorted by X.
                    //
                    for (int j = 0; j < targets.length; j++)
                    {
                        targets[j] = updatedRecognitions.get(j);
                    }

                    Arrays.sort(targets, this::compareTargetX);
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

            if (tracer != null)
            {
                for (int i = 0; i < targets.length; i++)
                {
                    tracer.traceInfo(funcName, "Minerals: [%d/%d: %s]: x=%.0f, y=%.0f, w=%.0f, h=%.0f",
                            i, targets.length, targets[i].getLabel(), targets[i].getTop(),
                            targets[i].getImageWidth() - targets[i].getRight(), targets[i].getHeight(),
                            targets[i].getWidth());
                }
            }
        }

        return targets;
    }   //getTargets

    public TargetInfo getTargetInfo(String label, int numExpectedTargets)
    {
        final String funcName = "getTargetInfo";
        TargetInfo targetInfo = null;
        Recognition[] targets = getTargets(label, numExpectedTargets);

        if (targets != null)
        {
            for (int i = 0; i < targets.length; i++)
            {
                if (targets[i].getLabel().equals(label))
                {
                    //
                    // Found the lowest target on the screen.
                    // Target object is presented in portrait mode but since the phone orientation is really landscape,
                    // we need to transpose the rect to landscape coordinates.
                    //
                    targetInfo = new TargetInfo(
                            targets[i].getLabel(),
                            new Rect(
                                    (int)targets[i].getTop(), (int)(targets[i].getImageWidth() - targets[i].getRight()),
                                    (int)targets[i].getHeight(), (int)targets[i].getWidth()),
                            targets[i].estimateAngleToObject(AngleUnit.DEGREES),
                            targets[i].getConfidence(),
                            targets[i].getImageHeight(), targets[i].getImageWidth());
                    //
                    // If we found the expected number of targets, the position is the index of the array.
                    // Otherwise, we will determine the position by the target's rect on the screen.
                    //
                    if (targets.length == numExpectedTargets)
                    {
                        targetInfo.position = i;
                    }
                    else
                    {
                        int xCenter = targetInfo.rect.x + targetInfo.rect.width/2;

                        if (xCenter < LEFT_THRESHOLD)
                        {
                            targetInfo.position = 0;
                        }
                        else if (xCenter < RIGHT_THRESHOLD)
                        {
                            targetInfo.position = 1;
                        }
                        else
                        {
                            targetInfo.position = 2;
                        }
                    }

                    if (tracer != null)
                    {
                        tracer.traceInfo(funcName, "###TargetInfo###: [%d/%d] %s",
                                i, targets.length, targetInfo);
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
        Recognition[] targets = getTargets(label, numExpectedTargets);

        if (targets != null)
        {
            targetsInfo = new TargetInfo[targets.length];
            for (int i = 0; i < targets.length; i++)
            {
                targetsInfo[i] = new TargetInfo(
                        targets[i].getLabel(),
                        new Rect(
                                (int)targets[i].getTop(), (int)(targets[i].getImageWidth() - targets[i].getRight()),
                                (int)targets[i].getHeight(), (int)targets[i].getWidth()),
                        targets[i].estimateAngleToObject(AngleUnit.DEGREES),
                        targets[i].getConfidence(),
                        targets[i].getImageHeight(), targets[i].getImageWidth());

                int xCenter = targetsInfo[i].rect.x + targetsInfo[i].rect.width/2;
                if (xCenter < LEFT_THRESHOLD)
                {
                    targetsInfo[i].position = 0;
                }
                else if (xCenter < RIGHT_THRESHOLD)
                {
                    targetsInfo[i].position = 1;
                }
                else
                {
                    targetsInfo[i].position = 2;
                }

                if (tracer != null)
                {
                    tracer.traceInfo(funcName, "###TargetInfo###: [%d/%d] %s",
                            i, targets.length, targetsInfo[i]);
                }
            }
        }

        return targetsInfo;
    }   //getAllTargetInfo

}   //class TensorFlowVision
