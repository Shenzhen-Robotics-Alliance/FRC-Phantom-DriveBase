package frc.robot.Utils.Tests;

import frc.robot.Drivers.IMUs.NavX2IMU;
import frc.robot.Drivers.IMUs.SimpleGyro;

public class Nav2XIMUTest implements SimpleRobotTest {
    private final SimpleGyro gyro = new SimpleGyro(0, true, new NavX2IMU());
    @Override
    public void testStart() {
        gyro.reset();
    }

    @Override
    public void testPeriodic() {
        System.out.println("gyro yaw: " + gyro.getYaw());
    }
}
