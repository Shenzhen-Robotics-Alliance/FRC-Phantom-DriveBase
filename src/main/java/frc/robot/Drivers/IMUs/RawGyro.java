package frc.robot.Drivers.IMUs;

public interface RawGyro {
    /** update the readings */
    void update();

    /** get the raw yaw pitch and roll angle, in radian */
    double[] getRawYawPitchRollAngle();

    /** in radian per second */
    double[] getYawYawPitchRollVelocity();
}
