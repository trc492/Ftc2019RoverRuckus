package tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import ftclib.FtcI2cDeviceSynch;
import ftclib.FtcOpMode;
import hallib.HalDashboard;
import trclib.TrcRobot;

@TeleOp(name="Test: FTC Raw Pixy Test", group="Test")
@Disabled
public class FtcTestPixyCamRaw extends FtcOpMode
{
    private HalDashboard dashboard;
    private FtcI2cDeviceSynch pixyCam;

    @Override
    public void initRobot()
    {
        dashboard = HalDashboard.getInstance();
        pixyCam = hardwareMap.get(FtcI2cDeviceSynch.class, "pixy");
        pixyCam.setDeviceInfo(HardwareDevice.Manufacturer.Other, "Pixy Camera v1");
        pixyCam.setI2cAddress(0x54, true);
    }   //initRobot

    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        pixyCam.getDeviceClient().engage();
    }   //startMode

    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        pixyCam.getDeviceClient().disengage();
    }   //stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        byte[] data = pixyCam.readData(0,14);
        if (data != null)
        {
            for (int i = 0; i + 1 < data.length; i += 2)
            {
                if (dashboard != null)
                dashboard.displayPrintf(i/2 + 1, "%02d: %02x, %02x", i, data[i], data[i + 1]);
            }
        }
    }   //runPeriodic

}   //FtcTestPixyCamRaw
