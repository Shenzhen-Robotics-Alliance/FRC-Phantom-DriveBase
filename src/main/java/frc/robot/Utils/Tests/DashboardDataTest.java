package frc.robot.Utils.Tests;


import frc.robot.Utils.EasyDataFlow;
import frc.robot.Utils.SequentialCommandFactory;


public class DashboardDataTest implements SimpleRobotTest {
    @Override
    public void testStart() {
        EasyDataFlow.putCurvesOnField("test/test curve", SequentialCommandFactory.getBezierCurvesFromPathFile("Leave Community"));
    }

    @Override
    public void testPeriodic() {
    }
}
