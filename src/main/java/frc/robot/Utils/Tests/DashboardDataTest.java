package frc.robot.Utils.Tests;


import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import org.littletonrobotics.junction.Logger;


public class DashboardDataTest implements SimpleRobotTest {
    private final StructArrayPublisher<Pose2d> pose2dStructArrayPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("trajectory", Pose2d.struct).publish();
    private final StructArrayPublisher<Pose3d> pose3dStructArrayPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("notes", Pose3d.struct).publish();

    @Override
    public void testStart() {

    }

    @Override
    public void testPeriodic() {
        Pose2d[] pose2ds = new Pose2d[] {
                new Pose2d(1, 2, Rotation2d.fromRadians(0)),
                new Pose2d(2, 2, Rotation2d.fromRadians(0)),
                new Pose2d(2, 3, Rotation2d.fromRadians(0)),
        };
        pose2dStructArrayPublisher.set(pose2ds);
        Logger.recordOutput("test1", pose2ds);

        final double noteHeight = 0.05;
        Pose3d[] pose3ds = new Pose3d[] {
                new Pose3d(new Translation3d(5, 5, noteHeight / 2), new Rotation3d(0, 0, 0)),
                new Pose3d(new Translation3d(4, 4, noteHeight / 2), new Rotation3d(0, 0, 0))
        };
        pose3dStructArrayPublisher.set(pose3ds);
        Logger.recordOutput("test2", pose3ds);
    }
}
