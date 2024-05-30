package frc.robot.Modules.PositionReader;

import frc.robot.Utils.MathUtils.AngleUtils;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;

public class PositionsEstimatorSimulation extends RobotFieldPositionEstimator {
    private Vector2D robotPosition, robotVelocity, robotAcceleration;
    private double robotFacing, robotAngularVelocity;
    public PositionsEstimatorSimulation() {}

    @Override
    public Vector2D getRobotVelocity2D() {
        return robotVelocity;
    }

    @Override
    public double getRobotRotationalVelocity() {
        return robotAngularVelocity;
    }

    @Override
    public Vector2D getRobotPosition2D() {
        return robotPosition;
    }

    @Override
    public Vector2D getRobotAcceleration2D() {
        return robotAcceleration;
    }

    @Override
    public double getRobotRotation() {
        return robotFacing;
    }

    @Override
    public void setRobotPosition(Vector2D robotPosition) {
        this.robotPosition = robotPosition;
    }

    public void setRobotStatus(Vector2D robotVelocity, Vector2D robotAcceleration) {
        this.robotVelocity = robotVelocity;
        this.robotAcceleration = robotAcceleration;
    }

    @Override
    public void setRobotRotation(double rotation) {
        this.robotFacing = rotation;
    }

    public void setRobotAngularVelocity(double angularVelocity) {
        this.robotAngularVelocity = angularVelocity;
    }

    @Override
    public void init() {}

    @Override
    protected void periodic(double dt) {
        this.robotFacing = AngleUtils.simplifyAngle(robotFacing + robotAngularVelocity * dt);
        this.robotPosition = robotPosition.addBy(robotVelocity.multiplyBy(dt));
    }

    @Override
    public void onReset() {
        this.robotFacing = RobotFieldPositionEstimator.toActualRobotRotation(new Rotation2D(Math.toRadians(90))).getRadian();
        this.robotPosition = RobotFieldPositionEstimator.toActualPositionOnField(new Vector2D(new double[] {3, 1.6}));
    }
}
