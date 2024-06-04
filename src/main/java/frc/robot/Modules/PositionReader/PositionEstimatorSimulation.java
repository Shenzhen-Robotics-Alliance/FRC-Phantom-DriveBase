package frc.robot.Modules.PositionReader;

import frc.robot.Utils.MathUtils.AngleUtils;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.PhysicsSimulation.AllRealFieldPhysicsSimulation;

public class PositionEstimatorSimulation extends RobotFieldPositionEstimator {
    private Vector2D robotPosition, robotVelocity, robotAcceleration;
    private double robotFacing, robotAngularVelocity;
    public final AllRealFieldPhysicsSimulation physicsSimulation;
    public final AllRealFieldPhysicsSimulation.HolomonicRobotPhysicsSimulation robotPhysicsSimulation;

    public PositionEstimatorSimulation(AllRealFieldPhysicsSimulation physicsSimulation, AllRealFieldPhysicsSimulation.RobotProfile robotProfile) {
        this.physicsSimulation = physicsSimulation;
        this.robotPhysicsSimulation = physicsSimulation.addRobot(
                new AllRealFieldPhysicsSimulation.HolomonicRobotPhysicsSimulation(robotProfile));
    }

    @Override
    public Vector2D getRobotVelocity2DToField() {
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
    public Vector2D getRobotAcceleration2DToField() {
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
        this.robotFacing = AngleUtils.simplifyAngle(robotFacingRadian);
        this.robotAngularVelocity = robotAngularVelocityRadPerSec;
    }

    @Override
    public void setRobotRotation(double rotation) {
        this.robotFacing = AngleUtils.simplifyAngle(rotation);
    }

    public void setRobotAngularVelocity(double angularVelocity) {
        this.robotAngularVelocity = angularVelocity;
    }

    @Override
    public void init() {}

    @Override
    public void onReset() {
        this.robotFacing = RobotFieldPositionEstimator.toActualRobotRotation(new Rotation2D(Math.toRadians(-90))).getRadian();
        this.robotAngularVelocity = 0;
        this.robotPosition = RobotFieldPositionEstimator.getRobotDefaultStartingPosition();
        this.robotVelocity = this.robotAcceleration = new Vector2D();
        robotPhysicsSimulation.reset(getRobotPosition2D(), getRobotRotation2D());
    }
}
