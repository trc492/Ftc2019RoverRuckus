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

import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.Locale;

import ftclib.FtcDcMotor;
import ftclib.FtcPixyCam;
import trclib.TrcPixyCam;
import trclib.TrcUtil;

public class PixyVision
{
    private static final int PIXYCAM_WIDTH = 320;
    private static final int PIXYCAM_HEIGHT = 200;
    private static final boolean debugEnabled = false;

    // If last target rect is this old, its stale data.
    private static final double LAST_TARGET_RECT_FRESH_DURATION_SECONDS = 0.1;  // 100 msec

    public class TargetInfo
    {
        public Rect rect;
        public double xDistance;
        public double yDistance;
        public double angle;

        public TargetInfo(Rect rect, double xDistance, double yDistance, double angle)
        {
            this.rect = rect;
            this.xDistance = xDistance;
            this.yDistance = yDistance;
            this.angle = angle;
        }   //TargetInfo

        public String toString()
        {
            return String.format(Locale.US, "Rect[%d,%d,%d,%d], xDistance=%.1f, yDistance=%.1f, angle=%.1f",
                    rect.x, rect.y, rect.width, rect.height, xDistance, yDistance, angle);
        }
    }   //class TargetInfo

    public enum Orientation
    {
        NORMAL_LANDSCAPE,
        CLOCKWISE_PORTRAIT,
        ANTICLOCKWISE_PORTRAIT,
        UPSIDEDOWN_LANDSCAPE
    }   //enum Orientation

    private static final double PIXY_DISTANCE_SCALE = 7.0*62.0;   //DistanceInInches*targetWidthdInPixels
    private static final double TARGET_SIDE_WIDTH = 2.0;
    private static final double TARGET_FACE_DIAGONAL = Math.sqrt(2.0)*TARGET_SIDE_WIDTH;
    private static final double TARGET_CUBE_DIAGONAL = Math.sqrt(3.0)*TARGET_SIDE_WIDTH;
    //
    // The target cube is seen by the camera at random viewing angle. This means the TARGET_WIDTH as seen by the
    // camera is between the min width which is TARGET_SIDE_WIDTH and the max width which is the farthest diagonal
    // of the cube (TARGET_CUBE_DIAGONAL). So we average TARGET_SIDE_WIDTH, TARGET_FACE_DIAGONAL and
    // TARGET_CUBE_DIAGONAL to be the TARGET_WIDTH.
    //
    private static final double TARGET_WIDTH_INCHES =
            (TARGET_SIDE_WIDTH + TARGET_FACE_DIAGONAL + TARGET_CUBE_DIAGONAL)/3.0;

    private FtcPixyCam pixyCamera;
    private Robot robot;
    private Orientation orientation;
    private FtcDcMotor light;
    private boolean lightOn = false;
    private Rect lastTargetRect = null;
    private double lastTargetRectExpireTime = TrcUtil.getCurrentTime();

    public PixyVision(String instanceName, Robot robot, Orientation orientation, int brightness)
    {
        pixyCamera = new FtcPixyCam(instanceName);
        this.robot = robot;
        this.orientation = orientation;
        light = new FtcDcMotor("pixyRingLight");
        setLightOn(false);
        pixyCamera.setBrightness((byte)brightness);
    }   //PixyVision

    public void setCameraEnabled(boolean enabled)
    {
        setLightOn(enabled);
        pixyCamera.setEnabled(enabled);
    }   //setCameraEnabled

    public void toggleCamera()
    {
        boolean enabled = pixyCamera.isEnabled();
        enabled = !enabled;
        setCameraEnabled(enabled);
    }   //toggleCamera

    public boolean isEnabled()
    {
        return pixyCamera.isEnabled();
    }   //isEnabled

    public void setLightOn(boolean enabled)
    {
        if (enabled)
        {
            light.set(1.0);
        }
        else
        {
            light.set(0.0);
        }
    }   //setLightOn

    public void toggleLight()
    {
        lightOn = !lightOn;
        setLightOn(lightOn);
    }   //toggleLight

    public boolean isLightOn()
    {
        return lightOn;
    }   //isLightOn

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
    private Rect getTargetRect(int signature)
    {
        final String funcName = "getTargetRect";
        Rect targetRect = null;
        TrcPixyCam.ObjectBlock[] detectedObjects = pixyCamera.getDetectedObjects();
        double currTime = TrcUtil.getCurrentTime();

        if (debugEnabled)
        {
            robot.globalTracer.traceInfo(
                    funcName, "%s object(s) found", detectedObjects != null? "" + detectedObjects.length: "null");
        }

        if (detectedObjects != null && detectedObjects.length > 0)
        {
            //
            // Make sure the camera detected at least one object.
            //
            ArrayList<Rect> objectList = new ArrayList<>();
            //
            // Filter out objects that don't have the correct signature.
            //
            for (int i = 0; i < detectedObjects.length; i++)
            {
                if (signature == detectedObjects[i].signature)
                {
                    int temp;
                    //
                    // If we have the camera mounted in other orientations, we need to adjust the object rectangles
                    // accordingly.
                    //
                    switch (orientation)
                    {
                        case CLOCKWISE_PORTRAIT:
                            temp = PIXYCAM_WIDTH - detectedObjects[i].centerX;
                            detectedObjects[i].centerX = detectedObjects[i].centerY;
                            detectedObjects[i].centerY = temp;
                            temp = detectedObjects[i].width;
                            detectedObjects[i].width = detectedObjects[i].height;
                            detectedObjects[i].height = temp;
                            break;

                        case ANTICLOCKWISE_PORTRAIT:
                            temp = detectedObjects[i].centerX;
                            detectedObjects[i].centerX = PIXYCAM_HEIGHT - detectedObjects[i].centerY;
                            detectedObjects[i].centerY = temp;
                            temp = detectedObjects[i].width;
                            detectedObjects[i].width = detectedObjects[i].height;
                            detectedObjects[i].height = temp;
                            break;

                        case UPSIDEDOWN_LANDSCAPE:
                            detectedObjects[i].centerX = PIXYCAM_WIDTH - detectedObjects[i].centerX;
                            detectedObjects[i].centerY = PIXYCAM_HEIGHT - detectedObjects[i].centerY;
                            break;

                        case NORMAL_LANDSCAPE:
                            break;
                    }

                    Rect rect = new Rect(detectedObjects[i].centerX - detectedObjects[i].width/2,
                            detectedObjects[i].centerY - detectedObjects[i].height/2,
                            detectedObjects[i].width, detectedObjects[i].height);
                    //
                    // The mineral will be at the lower screen of the camera. If we spot anything at the upper
                    // screen, they are considered false targets, so discard them.
                    //
                    if (rect.y >= (PIXYCAM_HEIGHT / 2))
                    {
                        objectList.add(rect);
                    }

                    if (debugEnabled)
                    {
                        robot.globalTracer.traceInfo(funcName, "[%d] %s", i, detectedObjects[i].toString());
                    }
                }
            }

            if (objectList.size() >= 1)
            {
                //
                // Find the largest target rect in the list.
                //
                Rect maxRect = objectList.get(0);
                double maxArea = maxRect.width * maxRect.height;

                for(int i = 1; i < objectList.size(); i++)
                {
                    Rect rect = objectList.get(i);
                    double area = rect.width * rect.height;
                    if (area > maxArea)
                    {
                        maxRect = rect;
                        maxArea = area;
                    }
                }

                targetRect = maxRect;

                if (debugEnabled)
                {
                    robot.globalTracer.traceInfo(funcName, "===TargetRect===: x=%d, y=%d, w=%d, h=%d",
                            targetRect.x, targetRect.y, targetRect.width, targetRect.height);
                }
            }

            if (targetRect == null)
            {
                robot.globalTracer.traceInfo(funcName, "===TargetRect=== None, is now null");
            }

            lastTargetRect = targetRect;
            lastTargetRectExpireTime = currTime + LAST_TARGET_RECT_FRESH_DURATION_SECONDS;
        }
        else if (currTime < lastTargetRectExpireTime)
        {
            targetRect = lastTargetRect;
        }

        return targetRect;
    }   //getTargetRect

    public TargetInfo getTargetInfo(int signature)
    {
        final String funcName = "getTargetInfo";
        TargetInfo targetInfo = null;
        Rect targetRect = getTargetRect(signature);

        if (targetRect != null)
        {
            //
            // Physical target width:           W
            // Physical target distance 1:      D1
            // Target pixel width at 20 inches: w1
            // Physical target distance 2:      D2
            // Target pixel width at 24 inches: w2
            // Camera lens focal length:        f
            //    W/D1 = w1/f and W/D2 = w2/f
            // => f = w1*D1/W and f = w2*D2/W
            // => w1*D1/W = w2*D2/W
            // => w1*D1 = w2*D2 = PIXY_DISTANCE_SCALE
            //
            // Screen center X:                 Xs
            // Target center X:                 Xt
            // Heading error:                   e
            // Turn angle:                      a
            //    tan(a) = e/f
            // => a = atan(e/f) and f = w1*D1/W
            // => a = atan((e*W)/(w1*D1))
            //
            double targetCenterX = targetRect.x + targetRect.width/2.0;
            double targetXDistance = (targetCenterX - PIXYCAM_WIDTH/2.0)*TARGET_WIDTH_INCHES/targetRect.width;
            double targetYDistance = PIXY_DISTANCE_SCALE/targetRect.width;
            double targetAngle = Math.toDegrees(Math.atan(targetXDistance/targetYDistance));
            targetInfo = new TargetInfo(targetRect, targetXDistance, targetYDistance, targetAngle);

            if (debugEnabled)
            {
                robot.globalTracer.traceInfo(
                        funcName, "###TargetInfo###: xDist=%.1f, yDist=%.1f, angle=%.1f",
                        targetXDistance, targetYDistance, targetAngle);
            }
        }

        return targetInfo;
    }   //getTargetInfo

}   // class PixyVision
