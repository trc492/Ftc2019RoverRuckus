package tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import common.TensorFlowVision;
import ftclib.FtcOpMode;
import hallib.HalDashboard;
import trclib.TrcDbgTrace;
import trclib.TrcRobot;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@TeleOp(name="Test: FTC Tensor Flow", group="Test")
//@Disabled
public class FtcTestTensorFlow extends FtcOpMode
{
    private static final boolean USE_TRACELOG = true;

    private HalDashboard dashboard;
    private TrcDbgTrace tracer;
    private TensorFlowVision tensorFlowVision;

    @Override
    public void initRobot()
    {
        dashboard = HalDashboard.getInstance();
        tracer = getGlobalTracer();
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        final VuforiaLocalizer.CameraDirection CAMERA_DIR = BACK;
        tensorFlowVision = new TensorFlowVision(tfodMonitorViewId, CAMERA_DIR, tracer);

        if (USE_TRACELOG)
        {
            tracer.openTraceLog("/sdcard/FIRST/tracelog", "TensorFlow");
            tracer.traceInfo("Init", "Starting logging...");
        }
    }   //initRobot

    @Override
    public void startMode(TrcRobot.RunMode runMode)
    {
        if (USE_TRACELOG)
        {
            tracer.setTraceLogEnabled(true);
        }
        tensorFlowVision.setEnabled(true);
    }   //startMode

    @Override
    public void stopMode(TrcRobot.RunMode runMode)
    {
        tensorFlowVision.shutdown();
        if (USE_TRACELOG)
        {
            tracer.setTraceLogEnabled(false);
            tracer.closeTraceLog();
        }
    }   //stopMode

    @Override
    public void runContinuous(double elapsedTime)
    {
        TensorFlowVision.TargetInfo targetInfo;

        targetInfo = tensorFlowVision.getTargetInfo(TensorFlowVision.LABEL_GOLD_MINERAL);
        tracer.traceInfo("TensorFlow", "[%.3f] Detected=%s",
                elapsedTime, Boolean.toString(targetInfo != null));
        if (targetInfo != null)
        {
            tracer.traceInfo("TensorFlow", "[%.3f] %s", elapsedTime, targetInfo);
            dashboard.displayPrintf(1, "%s", targetInfo);
        }

        targetInfo = tensorFlowVision.getTargetInfo(TensorFlowVision.LABEL_SILVER_MINERAL);
        if (targetInfo != null)
        {
            tracer.traceInfo("TensorFlow", "[%.3f] %s", elapsedTime, targetInfo);
            dashboard.displayPrintf(2, "%s", targetInfo);
        }
    }   //runContinuous

}   //class FtcTestTensorFlow
