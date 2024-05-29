package frc.robot.Modules.PositionReader;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Modules.RobotModuleBase;
import frc.robot.Utils.EasyDataFlow;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;

/**
 * definitions:
 * position:
 *  the position of the robot is defined in meter and in an X-Y plane where the origin is in the BOTTOM-LEFT corner of the field
 *  positive-x is rightwards and positive-y is upwards
 * rotation:
 *  the rotation is defined in radian, POSITIVE is COUNTER-CLOCKWISE and zero is pointing upwards in the field
 * */
public abstract class RobotFieldPositionEstimator extends RobotModuleBase {
    public static final Vector2D robotDefaultStartingPositionBlue = new Vector2D(new double[] {0.5, 4.1});
    public static final Rotation2D pilotFacingBlue = new Rotation2D(Math.toRadians(270));

    protected RobotFieldPositionEstimator() {
        super("Position-Estimator");
    }

    /** get the velocity of the robot */
    public abstract Vector2D getRobotVelocity2D();

    /** get the angular velocity (radian/second) of the robot, positive is counter-clockwise */
    public abstract double getRobotRotationalVelocity();

    /** get the position of the robot */
    public abstract Vector2D getRobotPosition2D();

    public abstract Vector2D getRobotAcceleration2D();

    /** get the current facing of the robot */
    public abstract double getRobotRotation();

    public Rotation2D getRobotRotation2D() {
        return new Rotation2D(getRobotRotation());
    }

    /** reset the position of the robot */
    public void resetRobotPosition() {
        setRobotPosition(toActualPositionOnField(robotDefaultStartingPositionBlue));
    }

    /** reset the rotation of the robot */
    public void resetRobotRotation() {
        setRobotRotation(toActualRobotRotation(pilotFacingBlue).getRadian());
    }

    /** reset the robot status (both position and rotation) */
    public void resetRobot() {
        this.resetRobotPosition();
        this.resetRobotRotation();
    }

    /** calibrates the robot position to a given position */
    public abstract void setRobotPosition(Vector2D robotPosition);

    /** calibrates the robot rotation to a given radian */
    public abstract void setRobotRotation(double rotation);

    /** calibrates the robot's status into a given status */
    public void setRobotStatus(Vector2D robotPosition, double rotation) {
        setRobotPosition(robotPosition);
        setRobotRotation(rotation);
    }

    public abstract void reset();

    @Override
    protected void periodic(double dt) {
        EasyDataFlow.putRobot(getRobotPosition2D(), getRobotRotation2D());
    }

    public static Rotation2D toActualRobotRotation(Rotation2D robotRotationAtBlueAlliance) {
        return switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)) {
            case Red -> new Rotation2D(robotRotationAtBlueAlliance.getRadian() + Math.PI);
            case Blue -> robotRotationAtBlueAlliance;
        };
    }
    public static Vector2D toActualPositionOnField(Vector2D positionOnFieldAtBlueAlliance) {
        final double fieldWidth = 16.54; // meters
        return switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)) {
            case Red -> new Vector2D(new double[] {fieldWidth - positionOnFieldAtBlueAlliance.getX(), positionOnFieldAtBlueAlliance.getY()});
            case Blue -> positionOnFieldAtBlueAlliance;
        };
    }
}
