package frc.robot.Modules.PositionReader;

import frc.robot.Utils.EasyDataFlow;
import frc.robot.Utils.MathUtils.AngleUtils;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.PhysicsSimulation.AllRealFieldPhysicsSimulation;
import frc.robot.Utils.RobotConfigReader;

public class PositionEstimatorSimulation extends RobotFieldPositionEstimator {
    private Vector2D robotPosition, robotVelocity, robotAcceleration;
    private double robotFacing, robotAngularVelocity;
    public final AllRealFieldPhysicsSimulation.HolomonicRobotPhysicsSimulation robotPhysicsSimulation;

    public PositionEstimatorSimulation(RobotConfigReader robotConfig) {
        final AllRealFieldPhysicsSimulation.RobotProfile robotProfile = new AllRealFieldPhysicsSimulation.RobotProfile(robotConfig);
        this.robotPhysicsSimulation = new AllRealFieldPhysicsSimulation.HolomonicRobotPhysicsSimulation(robotProfile);
    }

    @Override
    protected void periodic(double dt) {
        final Vector2D acceleration = Vector2D.displacementToTarget(getRobotVelocity2DToField(), robotPhysicsSimulation.getFieldVelocity()).multiplyBy(1.0/dt);
        updateRobotTranslationalStatus(robotPhysicsSimulation.getFieldPosition(), robotPhysicsSimulation.getFieldVelocity(), acceleration);

        updateRobotRotationalStatus(robotPhysicsSimulation.getFacing().getRadian(), robotPhysicsSimulation.getAngularVelocity());

        EasyDataFlow.putNumber("chassis physics simulation", "robot velocity (X)", getRobotVelocity2DToField().getX());
        EasyDataFlow.putNumber("chassis physics simulation", "robot velocity (Y)", getRobotVelocity2DToField().getY());
        EasyDataFlow.putNumber("chassis physics simulation", "robot facing (Deg)", Math.toDegrees(getRobotRotation()));
        EasyDataFlow.putNumber("chassis physics simulation", "robot angular velocity (Deg/Sec)", Math.toDegrees(getRobotRotationalVelocity()));

        EasyDataFlow.putRobot(robotPhysicsSimulation);
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
