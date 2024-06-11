package frc.robot.Utils.Tests;

import frc.robot.Utils.Alert;

public class DashboardDataTest implements SimpleRobotTest {
    @Override
    public void testStart() {
        final Alert alert = new Alert("Test Warning", Alert.AlertType.ERROR);
        alert.set(true);
    }

    @Override
    public void testPeriodic() {
    }
}
