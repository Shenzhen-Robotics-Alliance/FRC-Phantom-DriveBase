package frc.robot.Modules.Chassis;

import frc.robot.Modules.PositionReader.PositionEstimatorSimulation;
import frc.robot.Utils.PhysicsSimulation.AllRealFieldPhysicsSimulation;
import frc.robot.Utils.PhysicsSimulation.CollisionDetectionGrid;
import frc.robot.Utils.EasyDataFlow;
import frc.robot.Utils.RobotConfigReader;
import frc.robot.Utils.RobotModuleOperatorMarker;

public class SwerveDriveChassisSimulation extends SwerveDriveChassisLogic {
    private final PositionEstimatorSimulation positionEstimatorSimulation;
    private final CollisionDetectionGrid collisionDetectionGrid; // legacy
    private final AllRealFieldPhysicsSimulation physicsSimulation;
    private final AllRealFieldPhysicsSimulation.HolomonicRobotPhysicsSimulation chassisSimulation;
    public SwerveDriveChassisSimulation(SwerveWheelSimulation frontLeft, SwerveWheelSimulation frontRight, SwerveWheelSimulation backLeft, SwerveWheelSimulation backRight, PositionEstimatorSimulation positionEstimatorSimulation, AllRealFieldPhysicsSimulation physicsSimulation, RobotConfigReader robotConfig) {
        super(frontLeft, frontRight, backLeft, backRight, positionEstimatorSimulation, robotConfig);
        this.physicsSimulation = physicsSimulation;
        collisionDetectionGrid = new CollisionDetectionGrid();
        chassisSimulation = physicsSimulation.addRobot(positionEstimatorSimulation.robotPhysicsSimulation);
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
        chassisSimulation.simulateChassisRotationalBehavior(calculateRotationalPower(dt));
        chassisSimulation.simulateChassisTranslationalBehavior(calculateTranslationalPowerToRobot(dt), positionEstimator.getRobotRotation2D());
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

    @Override
    public void onReset() {
        super.onReset();
    }
}
