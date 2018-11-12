package tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

@TeleOp(name="Test: Sensor Loop Time", group="Test")
//@Disabled
public class TestSensorTime extends LinearOpMode
{
    private BNO055IMU imu;
    private DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException
    {
        double[] angles = new double[3];

        imu = hardwareMap.get(BNO055IMU.class,"imu");
        initIMU(imu);
        motor = hardwareMap.get(DcMotor.class, "lfWheel");

        waitForStart();

        while (opModeIsActive())
        {
            long startTime = System.currentTimeMillis();
            getEulerAngles(angles);
            System.out.printf("***** Elapsed time=%.3f (%.2f/%.2f/%.2f)\n",
                    (System.currentTimeMillis() - startTime)/1000.0,
                    angles[0], angles[1], angles[2]);

            startTime = System.currentTimeMillis();
            System.out.printf("***** Elapsed time=%.3f (pos=%d)\n",
                    (System.currentTimeMillis() - startTime)/1000.0,
                    motor.getCurrentPosition());
        }
    }

    public void initIMU(BNO055IMU imu)
    {
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();

        imuParams.mode = BNO055IMU.SensorMode.IMU;
        imuParams.useExternalCrystal = true;
        imuParams.pitchMode = BNO055IMU.PitchMode.WINDOWS;

        imuParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParams.loggingEnabled = true;
        imuParams.loggingTag = "IMU";
        imuParams.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(imuParams);
    }   //initialize

    private void getEulerAngles(double[] angles)
    {
        Quaternion q = imu.getQuaternionOrientation();
        //
        // 0: roll (x-axis rotation)
        // 1: pitch (y-axis rotation)
        // 2: yaw (z-axis rotation)
        //
        angles[0] = Math.toDegrees(Math.atan2(2.0*(q.w*q.x + q.y*q.z), 1.0 - 2.0*(q.x*q.x + q.y*q.y)));
        double sinp = 2.0*(q.w*q.y - q.z*q.x);
        angles[1] = Math.toDegrees(Math.abs(sinp) >= 1.0? Math.signum(sinp)*(Math.PI/2.0): Math.asin(sinp));
        angles[2] = Math.toDegrees(Math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z)));
    }   //getEulerAngles

}
