package frc.robot.Utils.Tests;

import frc.robot.Utils.EasyDataFlow;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;

public class DataFlowTest implements SimpleRobotTest {
    @Override
    public void testStart() {

    }

    @Override
    public void testPeriodic() {
        EasyDataFlow.putPositionArray(
                "position 1",
                new Vector2D[] {new Vector2D(new double[] {3, 3}), new Vector2D(new double[] {2.5, 2.5})},
                new Rotation2D[] {new Rotation2D(0), new Rotation2D(Math.PI)});
    }
}
