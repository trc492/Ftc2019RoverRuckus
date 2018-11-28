package team6541;

import ftclib.FtcDcMotor;

public class RearWheelMotor extends FtcDcMotor
{
    private FtcDcMotor frontWheelMotor;

    public RearWheelMotor(String instanceName, FtcDcMotor frontWheelMotor)
    {
        super(instanceName);
        this.frontWheelMotor = frontWheelMotor;
    }

    @Override
    public double getMotorPosition()
    {
        return frontWheelMotor.getMotorPosition();
    }
}   //RearWheelMotor
