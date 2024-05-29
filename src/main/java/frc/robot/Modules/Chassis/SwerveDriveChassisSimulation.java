package frc.robot.Modules.Chassis;

import frc.robot.Utils.EasyDataFlow;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.RobotModuleOperatorMarker;

public class SwerveDriveChassisSimulation extends HolonomicChassis {
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
        super.periodic(dt);
    }

    @Override
    public void onReset() {

    }
}
