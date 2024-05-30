package frc.robot.Modules.Chassis;

import frc.robot.Modules.PositionReader.PositionsEstimatorSimulation;
import frc.robot.Modules.PositionReader.RobotFieldPositionEstimator;
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
        switch (translationalTask.taskType) {

        }
        driveWheelsSafeLogic(translationalTask.translationValue, rotationalTask.rotationalValue);

        super.periodic(dt);
    }

    @Override
    public void onReset() {
        super.onReset();
    }
}
