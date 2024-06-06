package frc.robot.Utils.Tests;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Utils.EasyDataFlow;
import frc.robot.Utils.MathUtils.LookUpTable;
import frc.robot.Utils.MathUtils.Vector2D;

public class DashboardDataTest implements SimpleRobotTest {
    @Override
    public void testStart() {

    }

    private final XboxController xboxController = new XboxController(1);
    private double launchTime = 0;
    private static final double
            startingX = 3.6, startingY = 5.55, startingHeight = 0.4,
            speakerX = 0, speakerY = 5.55, speakerHeight = 2.2;
    @Override
    public void testPeriodic() {
        if (xboxController.getAButton())
            launchTime = Timer.getFPGATimestamp();
        final double
                x = LookUpTable.linearInterpretationWithBounding(launchTime, startingX, launchTime+0.5, speakerX, Timer.getFPGATimestamp()),
                y = LookUpTable.linearInterpretationWithBounding(launchTime, startingY, launchTime+0.5, speakerY, Timer.getFPGATimestamp()),
                z = LookUpTable.linearInterpretationWithBounding(launchTime, startingHeight, launchTime+0.5, speakerHeight, Timer.getFPGATimestamp());
        final Vector2D displacement2D = Vector2D.displacementToTarget(new Vector2D(new double[] {startingX, startingY}), new Vector2D(new double[] {speakerX, speakerY}));
        final double yaw = displacement2D.getHeading(),
                pitch = Math.atan2(speakerHeight - startingHeight, displacement2D.getMagnitude()); // TODO: bugs here

        EasyDataFlow.putPosition3dArray("test/flying notes", new Pose3d[] {new Pose3d(new Translation3d(x, y, z), new Rotation3d(0, pitch, yaw))});
    }
}
