package frc.robot.Modules.PositionReader;

import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;

public interface PositionEstimator {
    /** get the velocity of the robot */
    Vector2D getRobotVelocity2D();

    /** get the angular velocity (radian/second) of the robot, positive is counter-clockwise */
    double getRobotRotationalVelocity();

    /** get the position of the robot */
    Vector2D getRobotPosition2D();

    Vector2D getRobotAcceleration2D();

    /** get the current facing of the robot */
    double getRobotRotation();

    default Rotation2D getRobotRotation2D() {
        return new Rotation2D(getRobotRotation());
    }

    /** reset the position of the robot */
    void resetRobotPosition();

    /** reset the rotation of the robot */
    void resetRobotRotation();

    /** reset the robot status (both position and rotation) */
    default void resetRobot() {
        this.resetRobotPosition();
        this.resetRobotRotation();
    }

    /** calibrates the robot position to a given position */
    void setRobotPosition(Vector2D robotPosition);

    /** calibrates the robot rotation to a given radian */
    void setRobotRotation(double rotation);

    /** calibrates the robot's status into a given status */
    default void setRobotStatus(Vector2D robotPosition, double rotation) {
        setRobotPosition(robotPosition);
        setRobotRotation(rotation);
    }

    /** whether the last update is reliable(for some vision-calculator there might situations when the target is lost) */
    boolean isResultReliable();

    void reset();
}
