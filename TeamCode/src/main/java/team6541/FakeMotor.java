package team6541;

import ftclib.FtcDcMotor;

public class FakeMotor extends FtcDcMotor
{
    private FtcDcMotor realMotor;

    public FakeMotor(String instanceName, FtcDcMotor realMotor)
    {
        super(instanceName);
        this.realMotor = realMotor;
    }

    public double getPosition()
    {
        return realMotor.getPosition();
    }
}
