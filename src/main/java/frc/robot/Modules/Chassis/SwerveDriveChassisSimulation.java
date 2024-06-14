package frc.robot.Modules.Chassis;

import frc.robot.Modules.MatchFieldSimulation;
import frc.robot.Modules.PositionReader.PositionEstimatorSimulation;
import frc.robot.Utils.MechanismControllers.EnhancedPIDController;
import frc.robot.Utils.PhysicsSimulation.CollisionDetectionGrid;
import frc.robot.Utils.EasyDataFlow;
import frc.robot.Utils.RobotConfigReader;
import frc.robot.Utils.RobotModuleOperatorMarker;

public class SwerveDriveChassisSimulation extends SwerveDriveChassisLogic {
    private final PositionEstimatorSimulation positionEstimatorSimulation;
    private final CollisionDetectionGrid collisionDetectionGrid; // legacy
    private final MatchFieldSimulation simulation;
    public SwerveDriveChassisSimulation(RobotConfigReader robotConfig, SwerveWheelSimulation frontLeft, SwerveWheelSimulation frontRight, SwerveWheelSimulation backLeft, SwerveWheelSimulation backRight, PositionEstimatorSimulation positionEstimatorSimulation, MatchFieldSimulation matchFieldSimulation) {
        super(frontLeft, frontRight, backLeft, backRight, positionEstimatorSimulation, robotConfig);
        this.simulation = matchFieldSimulation;
        collisionDetectionGrid = new CollisionDetectionGrid();
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
        // override rotational controller
        super.goToRotationController = new EnhancedPIDController(new EnhancedPIDController.StaticPIDProfile(
                Math.PI * 2,
                0.6,
                0.05,
                Math.toRadians(35),
                Math.toRadians(1),
                0.1,
                0,
                0
        ));
    }

    @Override
    protected void periodic(double dt) {
        /* run swerve wheel simulation to simulate the behaviors of swerve wheels */
        driveWheelsSafeLogic(calculateTranslationalPowerToRobot(dt), calculateRotationalPower(dt));

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

        /* simulate chassis translation behavior */
        simulation.robotPhysicsSimulation.simulateChassisRotationalBehavior(calculateRotationalPower(dt));
        simulation.robotPhysicsSimulation.simulateChassisTranslationalBehavior(calculateTranslationalPowerToRobot(dt));
        simulation.updatePhysics(dt);
        super.periodic(dt);
    }

    @Override
    public void onReset() {
        super.onReset();
    }
}
