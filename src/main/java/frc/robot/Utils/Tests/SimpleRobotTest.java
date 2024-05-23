package frc.robot.Utils.Tests;

public interface SimpleRobotTest {
    void testStart();
    void testPeriodic();

    default void testEnd() {}
}
