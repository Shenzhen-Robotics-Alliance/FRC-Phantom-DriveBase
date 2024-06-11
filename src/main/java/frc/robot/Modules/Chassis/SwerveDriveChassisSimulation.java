package frc.robot.Modules.Chassis;

import frc.robot.Modules.PositionReader.PositionEstimatorSimulation;
import frc.robot.Utils.PhysicsSimulation.AllRealFieldPhysicsSimulation;
import frc.robot.Utils.PhysicsSimulation.CollisionDetectionGrid;
import frc.robot.Utils.EasyDataFlow;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.RobotConfigReader;
import frc.robot.Utils.RobotModuleOperatorMarker;
import org.dyn4j.dynamics.Force;

public class SwerveDriveChassisSimulation extends SwerveDriveChassisLogic {
    private final PositionEstimatorSimulation positionEstimatorSimulation;
    private final CollisionDetectionGrid collisionDetectionGrid;
    private final AllRealFieldPhysicsSimulation physicsSimulation;
    private final AllRealFieldPhysicsSimulation.HolomonicRobotPhysicsSimulation robotPhysicsSimulation;
    public SwerveDriveChassisSimulation(SwerveWheelLogic frontLeft, SwerveWheelLogic frontRight, SwerveWheelLogic backLeft, SwerveWheelLogic backRight, PositionEstimatorSimulation positionEstimatorSimulation, AllRealFieldPhysicsSimulation physicsSimulation, RobotConfigReader robotConfig) {
        super(frontLeft, frontRight, backLeft, backRight, positionEstimatorSimulation, robotConfig);
        this.physicsSimulation = physicsSimulation;
        collisionDetectionGrid = new CollisionDetectionGrid();
        robotPhysicsSimulation = physicsSimulation.addRobot(positionEstimatorSimulation.robotPhysicsSimulation);
        this.positionEstimatorSimulation = positionEstimatorSimulation;
    }

    @Override
    public double getChassisHeading() {
        return super.positionEstimator.getRobotRotation();
    }

    @Override
    public void setTranslationalTask(ChassisTaskTranslation translationalTask, RobotModuleOperatorMarker operator) {
        super.setTranslationalTask(translationalTask, operator);
    }

    @Override
    public void setRotationalTask(ChassisTaskRotation rotationalTask, RobotModuleOperatorMarker operator) {
        super.setRotationalTask(rotationalTask, operator);
    }

    @Override
    public boolean isCurrentTranslationalTaskFinished() {
        return false; // TODO: write this method
    }

    @Override
    public boolean isCurrentRotationalTaskFinished() {
        return false; // TODO: write this method
    }

    @Override
    public void resetChassisPositionAndRotation() {
        positionEstimatorSimulation.reset();
    }

    @Override
    public void updateConfigs() {
        super.updateConfigs();
    }

    @Override
    protected void periodic(double dt) {
        /* run swerve wheel simulation to simulate the behaviors of swerve wheels */
        driveWheelsSafeLogic(calculateTranslationalPowerToRobot(dt), calculateRotationalPower(dt));

        /* simulate chassis translation behavior */
        simulateChassisTranslationalBehavior(calculateTranslationalPowerToRobot(dt));
        /* simulate chassis rotation behavior */
        simulateChassisRotationalBehavior(calculateRotationalPower(dt));
        physicsSimulation.update(dt);

        /* simulate the swerve actual status from chassis motion */
        if (super.positionEstimator.getRobotVelocity2DToField().getMagnitude() == 0 && super.positionEstimator.getRobotRotationalVelocity() == 0)
            EasyDataFlow.putSwerveState(
                    "chassis/actual swerve state",
                    0, frontLeft.decideModuleDrivingDirection(),
                    0, frontRight.decideModuleDrivingDirection(),
                    0, backLeft.decideModuleDrivingDirection(),
                    0, backRight.decideModuleDrivingDirection(),
                    positionEstimator.getRobotRotation2D()
            );
        else
            EasyDataFlow.putSwerveState(
                    "chassis/actual swerve state",
                    frontLeft.calculateWheelMotion(positionEstimator.getRobotVelocity2DToRobot(), positionEstimator.getRobotRotationalVelocity()),
                    frontRight.calculateWheelMotion(positionEstimator.getRobotVelocity2DToRobot(), positionEstimator.getRobotRotationalVelocity()),
                    backLeft.calculateWheelMotion(positionEstimator.getRobotVelocity2DToRobot(), positionEstimator.getRobotRotationalVelocity()),
                    backRight.calculateWheelMotion(positionEstimator.getRobotVelocity2DToRobot(), positionEstimator.getRobotRotationalVelocity()),
                    positionEstimator.getRobotRotation2D()
            );
        super.periodic(dt);
    }

    private void simulateChassisTranslationalBehavior(Vector2D desiredMotionToRobot) {
        robotPhysicsSimulation.setAtRest(false);
        if (desiredMotionToRobot.getMagnitude() > 0.03)
            robotPhysicsSimulation.applyForce(new Force(Vector2D.toVector2(
                    desiredMotionToRobot.multiplyBy(positionEstimator.getRobotRotation2D()).multiplyBy(robotPhysicsSimulation.profile.propellingForce))));
        else {
            if (Vector2D.fromVector2(robotPhysicsSimulation.getLinearVelocity()).getMagnitude() > 0.03 * robotPhysicsSimulation.profile.robotMaxVelocity)
                robotPhysicsSimulation.applyForce(new Force(Vector2D.toVector2(
                        new Vector2D(Vector2D.fromVector2(robotPhysicsSimulation.getLinearVelocity()).getHeading(), -robotPhysicsSimulation.profile.frictionForce)
                )));
            else
                robotPhysicsSimulation.setLinearVelocity(0, 0);
        }
    }

    private void simulateChassisRotationalBehavior(double rotationPower) {
        EasyDataFlow.putNumber("chassis physics simulation", "desired rotational motion", rotationPower);
        if (Math.abs(rotationPower) > 0.05)
            robotPhysicsSimulation.applyTorque(rotationPower * robotPhysicsSimulation.profile.maxAngularAcceleration * robotPhysicsSimulation.getMass().getInertia());
        else {
            if (Math.abs(robotPhysicsSimulation.getAngularVelocity()) < robotPhysicsSimulation.profile.maxAngularVelocity * 0.05)
                robotPhysicsSimulation.setAngularVelocity(0);
            else
                robotPhysicsSimulation.applyTorque(Math.copySign(robotPhysicsSimulation.profile.angularFrictionAcceleration * robotPhysicsSimulation.getMass().getInertia(), -robotPhysicsSimulation.getAngularVelocity()));
        }
    }

    @Override
    public void onReset() {
        super.onReset();
    }
}
