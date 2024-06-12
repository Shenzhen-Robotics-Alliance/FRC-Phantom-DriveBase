package frc.robot.Modules.Chassis;

import frc.robot.Modules.PositionReader.RobotFieldPositionEstimator;
import frc.robot.Utils.*;
import frc.robot.Utils.MathUtils.Vector2D;

/**
 * the module that controls the chassis with its four swerves
 */
public class SwerveDriveChassis extends SwerveDriveChassisLogic {

    public SwerveDriveChassis(SwerveWheelLogic frontLeft, SwerveWheelLogic frontRight, SwerveWheelLogic backLeft, SwerveWheelLogic backRight, RobotFieldPositionEstimator positionEstimator, RobotConfigReader robotConfig) {

        super(frontLeft, frontRight, backLeft, backRight, positionEstimator, robotConfig);
    }

    @Override
    protected void periodic(double dt) {
        Vector2D processedTranslationalSpeed = calculateTranslationalPowerToRobot(dt);
        double rotationalSpeed = calculateRotationalPower(dt);

        driveWheelsSafeLogic(processedTranslationalSpeed, rotationalSpeed);

        displayActualSwerveStates();
        displayErrors();
        super.periodic(dt);
    }

    private void displayErrors() {
        super.imuError.set(!positionEstimator.imuCommunicationHealthy());

        super.frontLeftModuleError.set(frontLeft.getError());
        super.frontLeftModuleError.setText(frontLeft.getErrorMessage("Front-Left"));

        super.frontRightModuleError.set(frontRight.getError());
        super.frontRightModuleError.setText(frontLeft.getErrorMessage("Front-Right"));

        super.backLeftModuleError.set(backLeft.getError());
        super.backLeftModuleError.setText(frontLeft.getErrorMessage("Back-Left"));

        super.backRightModuleError.set(backRight.getError());
        super.backRightModuleError.setText(frontLeft.getErrorMessage("Back-Right"));
    }

    private void displayActualSwerveStates() {
        EasyDataFlow.putSwerveState(
                "chassis/actual swerve state",
                frontLeft.getActualSwerveState()[0],
                frontLeft.getActualSwerveState()[1],
                frontRight.getActualSwerveState()[0],
                frontRight.getActualSwerveState()[1],
                backLeft.getActualSwerveState()[0],
                backLeft.getActualSwerveState()[1],
                backRight.getActualSwerveState()[0],
                backRight.getActualSwerveState()[1],
                positionEstimator.getRobotRotation2D()
        );
    }

    @Override
    public void updateConfigs() {
        super.updateConfigs();
    }

    @Override
    public void onReset() {
        System.out.println("<-- chassis reset coming through --> ");
        super.onReset();
    }

    @Override
    public double getChassisHeading() {
        return positionEstimator.getRobotRotation();
    }

    @Override
    public void resetChassisPositionAndRotation() {
        positionEstimator.reset();
    }
}
