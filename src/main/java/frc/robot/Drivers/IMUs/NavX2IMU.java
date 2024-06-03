package frc.robot.Drivers.IMUs;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Utils.MathUtils.AngleUtils;

public class NavX2IMU implements RawGyro {
    private final AHRS ahrs = new AHRS(SerialPort.Port.kMXP);
    @Override
    public void update() {

    }

    @Override
    public double[] getRawYawPitchRollAngle() {
        return new double[] {
                AngleUtils.simplifyAngle(Math.toRadians(ahrs.getYaw())),
                AngleUtils.simplifyAngle(Math.toRadians(ahrs.getPitch())),
                AngleUtils.simplifyAngle(Math.toRadians(ahrs.getRoll()))
        };
    }

    @Override
    public double[] getYawYawPitchRollVelocity() {
        return new double[] {
                Math.toRadians(ahrs.getRate()),
                0, 0
        };
    }
}
