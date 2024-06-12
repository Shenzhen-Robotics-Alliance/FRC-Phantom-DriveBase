package frc.robot.Utils.Tests;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Utils.Alert;

public class DashboardDataTest implements SimpleRobotTest {
    final Alert testError = new Alert("Test Error", Alert.AlertType.ERROR),
        testError2 = new Alert("Test Error 2", Alert.AlertType.ERROR),
        testWarning = new Alert("Test Warning", Alert.AlertType.WARNING),
        testInfo = new Alert("Test Message", Alert.AlertType.INFO);
    @Override
    public void testStart() {
        testError.setActivated(true);
        testError2.setActivated(true);
        testWarning.setActivated(true);
        testInfo.setActivated(true);
    }

    @Override
    public void testPeriodic() {
        testError2.setText("Test Error 2, time stamp: " + Timer.getFPGATimestamp());
    }
}
