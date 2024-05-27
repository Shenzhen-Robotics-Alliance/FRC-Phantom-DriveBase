package frc.robot.Utils.Tests;

import frc.robot.Utils.EasyDataFlow;

public class DataFlowTest implements SimpleRobotTest {
    @Override
    public void testStart() {

    }

    double i = 0;
    @Override
    public void testPeriodic() {
        i += 0.01;
        EasyDataFlow.putNumber("test", "test value", i);
    }
}
