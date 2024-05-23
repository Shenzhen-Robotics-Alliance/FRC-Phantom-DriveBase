package frc.robot.Modules.PositionReader;

import frc.robot.Drivers.IMUs.SimpleGyro;
import frc.robot.Drivers.Visions.PhantomClient;
import frc.robot.Modules.Chassis.SwerveWheel;

/**
 * TODO: the vision odometer that pulls results from phantom client and calibrates the chassis's position
 * */
public class VisionSupportedOdometer extends SwerveDriveOdometer {
    private final PhantomClient phantomClient;
    /**
     * public RobotModule(HashMap<String, RobotModule> dependenciesModules,
     * dependency object 1, dependency object 2, ...)
     *
     * @param swerveWheels
     * @param gyro
     */
    public VisionSupportedOdometer(SwerveWheel[] swerveWheels, SimpleGyro gyro, PhantomClient phantomClient) {
        super(swerveWheels, gyro);
        this.phantomClient = phantomClient;
    }
}
