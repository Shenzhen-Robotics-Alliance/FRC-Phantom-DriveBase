package frc.robot.Modules.PositionReader;

import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;

public class PositionEstimatorSimulation extends RobotFieldPositionEstimator {
    private Vector2D robotPosition, robotVelocity, robotAcceleration;
    private double robotFacing, robotAngularVelocity;
    public PositionEstimatorSimulation() {}

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

    public void updateRobotTranslationalStatus(Vector2D robotPosition, Vector2D robotVelocity, Vector2D robotAcceleration) {
        this.robotPosition = robotPosition;
        this.robotVelocity = robotVelocity;
        this.robotAcceleration = robotAcceleration;
    }

    public void updateRobotRotationalStatus(double robotFacingRadian, double robotAngularVelocityRadPerSec) {
        this.robotFacing = robotFacingRadian;
        this.robotAngularVelocity = robotAngularVelocityRadPerSec;
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
    }

    @Override
    public void onReset() {
        this.robotFacing = RobotFieldPositionEstimator.toActualRobotRotation(new Rotation2D(Math.toRadians(90))).getRadian();
        this.robotAngularVelocity = 0;
        this.robotPosition = RobotFieldPositionEstimator.toActualPositionOnField(new Vector2D(new double[] {3, 1.6}));
        this.robotVelocity = this.robotAcceleration = new Vector2D();
    }
}
