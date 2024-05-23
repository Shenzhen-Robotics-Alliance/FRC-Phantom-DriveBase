package frc.robot.AutoStagePrograms;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;

public class FieldPositions {
    public static final Vector2D
            nearNote1 = new Vector2D(new double[] {0, 2.9}),
            nearNote2 = new Vector2D(new double[] {1.45, 2.9}),
            nearNote3 = new Vector2D(new double[] {2.9, 2.9}),
            farNoteCenter = new Vector2D(new double[] {0, 8.27}),
            farNoteLefter = new Vector2D(new double[] {-1.68, 8.27}),
            farNoteRighter = new Vector2D(new double[] {1.68, 8.27}),
            farNoteLeftMost = new Vector2D(new double[] {-1.68*2, 8.27}),
            farNoteRightMost = new Vector2D(new double[] {1.68*2, 8.27}),
            speakerPosition = new Vector2D(new double[] {1.45, 0.2});

    public static Vector2D toActualPosition(Vector2D position) {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red ? position : new Vector2D(new double[] {-position.getX(), position.getY()});
    }

    public static Rotation2D toActualRotation(Rotation2D rotation2D) {
        System.out.println("to actual rotation alliance: " + DriverStation.getAlliance().orElse(DriverStation.Alliance.Red).name());
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red ? rotation2D : new Rotation2D(-rotation2D.getRadian());
    }
}
