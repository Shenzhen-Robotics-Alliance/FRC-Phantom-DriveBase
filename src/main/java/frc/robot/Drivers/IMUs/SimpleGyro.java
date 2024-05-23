package frc.robot.Drivers.IMUs;

import frc.robot.Utils.MathUtils.AngleUtils;

/**
 * simple gyro that can be used in 2d navigation only all the imu must be installed straightly
 * */
public class SimpleGyro {
    private double calibratedYawAngle = 0;
    /** to use which axis as yaw */
    private final int yawAxis;
    private final double yawRate;
    private final RawGyro gyroInstance;


    public SimpleGyro(int yawAxis, boolean reversed, RawGyro gyroInstance) {
        this.yawAxis = yawAxis;
        this.yawRate = reversed ? -1: 1;
        this.gyroInstance = gyroInstance;
    }

    /** reset yaw, pitch row to zero */
    public void reset() {
        calibrate(0);
    }

    /**
     * set the current yaw angle as given angle (and then automatically updates the value)
     * @param yawValue the given current yaw angle, in radians
     */
    public void calibrate(double yawValue) {
        calibratedYawAngle = AngleUtils.getActualDifference(yawValue, getRawYawValue());
    }

    /**
     * gets the yaw value calculated by the robot
     * @return the yaw angle. in radian, counter-clockwise is positive
     *  */
    public double getYaw() {
        return AngleUtils.simplifyAngle(AngleUtils.getActualDifference(calibratedYawAngle,getRawYawValue()));
    }

    public double getRawYawValue() {
        return AngleUtils.simplifyAngle(gyroInstance.getRawYawPitchRollAngle()[yawAxis] * yawRate);
    }


    /**
     *  gets the yaw angular velocity
     * @return the yaw angular velocity of the robot, in radian per second, counter-clockwise is positive
     *  */
    public double getYawVelocity() {
        return gyroInstance.getYawYawPitchRollVelocity()[yawAxis] * yawRate;
    }

    /**
     * updates the sensor reading and inertial navigation
     */
    public void update() {
        gyroInstance.update();
    }
}
