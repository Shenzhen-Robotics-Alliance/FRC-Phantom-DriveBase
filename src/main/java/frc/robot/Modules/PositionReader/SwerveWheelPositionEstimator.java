package frc.robot.Modules.PositionReader;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drivers.IMUs.SimpleGyro;
import frc.robot.Modules.RobotModuleBase;
import frc.robot.Modules.Chassis.SwerveWheel;
import frc.robot.Utils.ChassisUnit;
import frc.robot.Utils.EasyShuffleBoard;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;

public class SwerveWheelPositionEstimator extends RobotModuleBase implements PositionEstimator {
    protected final SwerveWheel[] swerveWheels;
    protected final SimpleGyro gyro;

    protected final Vector2D[] wheelPositions;
    protected final Vector2D[] wheelVelocities;
    protected final Vector2D[] wheelAccelerations;
    private final double[] encodersPreviousReadings;
    protected Timer dt = new Timer();

    /**
     * public RobotModule(HashMap<String, RobotModule> dependenciesModules,
     * dependency object 1, dependency object 2, ...)
     */
    public SwerveWheelPositionEstimator(SwerveWheel[] swerveWheels, SimpleGyro gyro) {
        super("Position-Estimator");
        this.swerveWheels = swerveWheels;
        this.wheelAccelerations = new Vector2D[swerveWheels.length];
        this.wheelVelocities = new Vector2D[swerveWheels.length];
        this.wheelPositions = new Vector2D[swerveWheels.length];
        this.encodersPreviousReadings = new double[swerveWheels.length];
        this.gyro = gyro;
    }

    @Override
    public void init() {
        onReset();
    }

    @Override
    protected void periodic(double dt) {
        for (int wheelID = 0; wheelID < swerveWheels.length; wheelID++) {
            Vector2D wheelVelocity = swerveWheels[wheelID].getModuleVelocity2D(); // gain the velocity of each module, in vector
            wheelVelocity = wheelVelocity.multiplyBy(new Rotation2D(gyro.getYaw())); // apply rotation according to the imu

            wheelPositions[wheelID] = wheelPositions[wheelID].addBy(
                    new Vector2D(swerveWheels[wheelID].getSteerHeading() + gyro.getYaw(), swerveWheels[wheelID].getWheelDrivingEncoderValue(ChassisUnit.METER) - encodersPreviousReadings[wheelID]));
            encodersPreviousReadings[wheelID] = swerveWheels[wheelID].getWheelDrivingEncoderValue(ChassisUnit.METER);

            wheelAccelerations[wheelID] = wheelVelocity
                    .addBy(wheelVelocities[wheelID].multiplyBy(-1))
                    .multiplyBy(1.0f/dt); // apply derivative to time

            wheelVelocities[wheelID] = wheelVelocity; // keep a copy of the velocity
        }

        EasyShuffleBoard.putNumber("chassis", "old position estimator (x)", getRobotPosition2D().getX());
        EasyShuffleBoard.putNumber("chassis", "old position estimator (y)", getRobotPosition2D().getY());
    }

    @Override
    public void onReset() {
        this.resetRobotPosition();
        this.resetRobotRotation();
        for (int wheelID = 0; wheelID < swerveWheels.length; wheelID++) {
            wheelVelocities[wheelID] = new Vector2D();
            wheelAccelerations[wheelID] = new Vector2D();
            encodersPreviousReadings[wheelID] = swerveWheels[wheelID].getWheelDrivingEncoderValue(ChassisUnit.METER);
        }
        dt.start();
        dt.reset();
    }

    @Override
    public Vector2D getRobotVelocity2D() {
        Vector2D robotVelocity = new Vector2D();
        for (int wheelID = 0; wheelID < swerveWheels.length; wheelID++)
            robotVelocity = robotVelocity.addBy(
                    wheelVelocities[wheelID].multiplyBy(1.0f / swerveWheels.length)); // take the average of the four wheel's velocity
        return robotVelocity;
    }

    @Override
    public double getRobotRotationalVelocity() {
        return gyro.getYawVelocity();
    }

    @Override
    public Vector2D getRobotPosition2D() {
        Vector2D robotPosition = new Vector2D();
        for (int wheelID = 0; wheelID < swerveWheels.length; wheelID++)
            robotPosition = robotPosition.addBy(
                    wheelPositions[wheelID].multiplyBy(1.0f / swerveWheels.length)); // take the average of the four wheel's velocity
        return robotPosition;
    }

    @Override
    public Vector2D getRobotAcceleration2D() {
        Vector2D robotAcceleration = new Vector2D();
        for (int wheelID = 0; wheelID < swerveWheels.length; wheelID++)
            robotAcceleration = robotAcceleration.addBy(
                    wheelAccelerations[wheelID].multiplyBy(1.0f / swerveWheels.length)); // take the average of the four wheel's velocity
        return robotAcceleration;
    }

    public double getDeviation() {
        double squaredDeviation = 0;
        for (int wheelID = 0; wheelID < swerveWheels.length; wheelID++) {
            Vector2D deviation = wheelPositions[wheelID].addBy(
                    swerveWheels[wheelID].getWheelInstalledLocationVector().multiplyBy(-1));
            squaredDeviation += deviation.getMagnitude() * deviation.getMagnitude();
        }
        squaredDeviation /= swerveWheels.length;
        return Math.sqrt(squaredDeviation);
    }

    @Override
    public double getRobotRotation() {
        return gyro.getYaw();
    }

    @Override
    public void resetRobotPosition() {
        setRobotPosition(new Vector2D());
    }

    @Override
    public void resetRobotRotation() {
        gyro.reset();
    }

    @Override
    public void setRobotPosition(Vector2D robotPosition) {
        for (int wheelID = 0; wheelID < swerveWheels.length; wheelID++)
            wheelPositions[wheelID] = robotPosition.addBy(swerveWheels[wheelID].getWheelInstalledLocationVector());
    }

    @Override
    public void setRobotRotation(double rotation) {
        gyro.calibrate(rotation);
    }

    @Override
    public boolean isResultReliable() {
        return true; // TODO determine whether it is reliable or not
    }
}
