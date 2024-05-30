package frc.robot.Modules.Chassis;

import frc.robot.Modules.PositionReader.RobotFieldPositionEstimator;
import frc.robot.Utils.RobotConfigReader;
import frc.robot.Utils.RobotModuleOperatorMarker;

public class SwerveDriveChassisSimulation extends SwerveDriveChassisLogic {
    public SwerveDriveChassisSimulation(SwerveWheelLogic frontLeft, SwerveWheelLogic frontRight, SwerveWheelLogic backLeft, SwerveWheelLogic backRight, RobotFieldPositionEstimator positionEstimator, RobotConfigReader robotConfig) {

        super(frontLeft, frontRight, backLeft, backRight, positionEstimator, robotConfig);
    }

    @Override
    public double getChassisHeading() {
        return 0;
    }

    @Override
    public ChassisTaskTranslation getCurrentTranslationalTask() {
        return null;
    }

    @Override
    public ChassisTaskRotation getCurrentRotationalTask() {
        return null;
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
        // TODO: this only works for manual operation, not go-to-position controls
        driveWheelsSafeLogic(translationalTask.translationValue, rotationalTask.rotationalValue);

        super.periodic(dt);
    }

    @Override
    public void onReset() {
        super.onReset();
    }
}
