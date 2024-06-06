package frc.robot.Utils.Tests;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DashboardDataTest implements SimpleRobotTest {
    private final Field2d field = new Field2d();
    @Override
    public void testStart() {
        field.getObject("Note").setPose(2, 3, Rotation2d.fromRadians(0));
        SmartDashboard.putData("test field", field);
    }

    @Override
    public void testPeriodic() {
    }
}
