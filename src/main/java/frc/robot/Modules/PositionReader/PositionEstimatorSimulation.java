package frc.robot.Modules.PositionReader;

import frc.robot.Utils.EasyDataFlow;
import frc.robot.Utils.MathUtils.AngleUtils;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.PhysicsSimulation.AllRealFieldPhysicsSimulation;

public class PositionEstimatorSimulation extends RobotFieldPositionEstimator {
    private Vector2D robotPosition, robotVelocity, robotAcceleration;
    private double robotFacing, robotAngularVelocity;

    private final AllRealFieldPhysicsSimulation.HolomonicRobotPhysicsSimulation robotPhysicsSimulation;
    public PositionEstimatorSimulation(AllRealFieldPhysicsSimulation.HolomonicRobotPhysicsSimulation robotPhysicsSimulation) {
        this.robotPhysicsSimulation = robotPhysicsSimulation;
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
        robotPhysicsSimulation.setRobotPosition(robotPosition);
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
        this.robotPhysicsSimulation.setRobotRotation(new Rotation2D(rotation));
    }

    public void setRobotAngularVelocity(double angularVelocity) {
        this.robotAngularVelocity = angularVelocity;
    }

    @Override
    public void init() {}

    @Override
    public void onReset() {
        robotPhysicsSimulation.resetMotion();
        robotVelocity = new Vector2D();
        robotAngularVelocity = 0;
        setRobotRotation(RobotFieldPositionEstimator.getPilotFacing2D().getRadian());
        setRobotPosition(RobotFieldPositionEstimator.getRobotDefaultStartingPosition());
    }
}
