package tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftclib.FtcOpMode;
import ftclib.FtcPixyCam;
import hallib.HalDashboard;
import trclib.TrcDbgTrace;
import trclib.TrcPixyCam;
import trclib.TrcRobot;

@TeleOp(name="Test: FTC Pixy Test", group="Test")
@Disabled
public class FtcTestPixyCam extends FtcOpMode
{
    private static final boolean USE_TRACELOG = true;

    private HalDashboard dashboard;
    private FtcPixyCam pixyCam;
    private TrcDbgTrace tracer;

    @Override
    public void initRobot()
    {
        dashboard = HalDashboard.getInstance();
        pixyCam = new FtcPixyCam("pixy");
        tracer = FtcOpMode.getGlobalTracer();

        if (USE_TRACELOG)
        {
            tracer.openTraceLog("/sdcard/FIRST/tracelog", "PixyCam");
            tracer.traceInfo("Init", "Starting logging...");
        }
    }   //initRobot

    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        if (USE_TRACELOG)
        {
            tracer.setTraceLogEnabled(true);
        }
        pixyCam.setEnabled(true);
    }

    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        pixyCam.setEnabled(false);

        if (USE_TRACELOG)
        {
            tracer.setTraceLogEnabled(false);
            tracer.closeTraceLog();
        }
    }

    @Override
    public void runPeriodic(double elapsedTime)
    {
        TrcPixyCam.ObjectBlock[] objects = pixyCam.getDetectedObjects();

        if (objects != null)
        {
            dashboard.displayPrintf(1, "Detected objects=%d.", objects.length);
            for (int i = 0; i < objects.length; i++)
            {
                dashboard.displayPrintf(2 + i, "%d: %d,%d(%d,%d)/%d",
                        objects[i].signature,
                        objects[i].centerX, objects[i].centerY, objects[i].width,
                        objects[i].height, objects[i].angle);
            }
        }
    }   //runPeriodic

}   //FtcTestPixyCam
