package frc.robot.Modules.Chassis;

import frc.robot.Modules.PositionReader.PositionsEstimatorSimulation;
import frc.robot.Utils.EasyDataFlow;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.RobotConfigReader;
import frc.robot.Utils.RobotModuleOperatorMarker;

public class SwerveDriveChassisSimulation extends SwerveDriveChassisLogic {
    private final PositionsEstimatorSimulation positionsEstimatorSimulation;
    public SwerveDriveChassisSimulation(SwerveWheelLogic frontLeft, SwerveWheelLogic frontRight, SwerveWheelLogic backLeft, SwerveWheelLogic backRight, PositionsEstimatorSimulation positionEstimatorSimulation, RobotConfigReader robotConfig) {

        super(frontLeft, frontRight, backLeft, backRight, positionEstimatorSimulation, robotConfig);
        this.positionsEstimatorSimulation = positionEstimatorSimulation;
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
    public void setChassisLocked(boolean locked, RobotModuleOperatorMarker operator) {

    }

    @Override
    public void setOrientationMode(OrientationMode mode, RobotModuleOperatorMarker operator) {

    }

    @Override
    public void setLowSpeedModeEnabled(boolean enabled, RobotModuleOperatorMarker operator) {

    }

    @Override
    public boolean isCurrentTranslationalTaskFinished() {
        return false;
    }

    @Override
    public boolean isCurrentRotationalTaskFinished() {
        return false;
    }

    @Override
    public void resetChassisPositionAndRotation() {

    }

    @Override
    public void init() {

    }

    @Override
    protected void periodic(double dt) {
        /* run swerve wheel simulation to simulate the behaviors of swerve wheels */
        driveWheelsSafeLogic(translationalTask.translationValue, rotationalTask.rotationalValue);

        /* simulate chassis translation behavior */
        switch (translationalTask.taskType) {
            case SET_VELOCITY -> simulateChassisBehaviorSetVelocity();
            case GO_TO_POSITION -> simulateChassisBehaviorGoToPosition();
        }
        /* simulate chassis rotation behavior */
        switch (rotationalTask.taskType) {
            case SET_VELOCITY -> simulateChassisBehaviorSetRotationalVelocity();
            case FACE_DIRECTION -> simulateChassisBehaviorFaceDirection();
        }


        final double defaultSwerveDirection = Math.toRadians(90);
        if (super.positionEstimator.getRobotVelocity2D().getMagnitude() != 0 && super.positionEstimator.getRobotRotationalVelocity() != 0)
            EasyDataFlow.putSwerveState(
                    "actual swerve state",
                    frontLeft.calculateWheelMotion(positionEstimator.getRobotVelocity2D(), positionEstimator.getRobotRotationalVelocity()),
                    frontRight.calculateWheelMotion(positionEstimator.getRobotVelocity2D(), positionEstimator.getRobotRotationalVelocity()),
                    backLeft.calculateWheelMotion(positionEstimator.getRobotVelocity2D(), positionEstimator.getRobotRotationalVelocity()),
                    backRight.calculateWheelMotion(positionEstimator.getRobotVelocity2D(), positionEstimator.getRobotRotationalVelocity()),
                    positionEstimator.getRobotRotation2D()
            );
        else
            EasyDataFlow.putSwerveState(
                    "actual swerve state",
                    0, defaultSwerveDirection,
                    0 ,defaultSwerveDirection ,
                    0 ,defaultSwerveDirection,
                    0 ,defaultSwerveDirection,
                    positionEstimator.getRobotRotation2D()
            );
        super.periodic(dt);
    }

    private void simulateChassisBehaviorGoToPosition() {
        // TODO simulate how the chassis will behave when asked to go to a position

    }

    private void simulateChassisBehaviorSetVelocity() {
        // TODO simulate how the chassis will behave when asked to move in a given translational velocity, in reference to ConceptSwerveDrivePhysicsSimulation
    }

    private void simulateChassisBehaviorFaceDirection() {
        // TODO simulate how the chassis will behave when asked to face a direction
    }

    private void simulateChassisBehaviorSetRotationalVelocity() {
        // TODO complete this using robotFacing = AngleUtils.simplifyAngle(robotFacing + robotAngularVelocity * dt);
    }

    @Override
    public void onReset() {
        super.onReset();
    }
}
