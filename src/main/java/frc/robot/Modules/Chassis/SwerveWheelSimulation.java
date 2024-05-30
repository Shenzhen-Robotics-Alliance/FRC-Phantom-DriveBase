package frc.robot.Modules.Chassis;

import frc.robot.Utils.ChassisUnit;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.RobotConfigReader;

public class SwerveWheelSimulation extends SwerveWheelLogic {
    public SwerveWheelSimulation(int swerveWheelID, RobotConfigReader robotConfig, Vector2D wheelPositionVector) {
        super(swerveWheelID, robotConfig, wheelPositionVector);
    }

    @Override
    protected void periodic(double dt) {
        super.actualDriveMotorPower = targetedSpeed;
        super.periodic(dt);
    }

    @Override
    public double getWheelDrivingEncoderValue(ChassisUnit unit) {
        return 0;
    }

    @Override
    public double getSteerHeading() {
        return commandedHeading;
    }

    @Override
    public Vector2D getModuleVelocity2D(ChassisUnit unit) {
        return new Vector2D(commandedHeading, targetedSpeed);
    }
}
