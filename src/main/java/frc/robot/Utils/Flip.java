package frc.robot.Utils;

import frc.robot.Utils.MathUtils.AngleUtils;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;

public class Flip {
    public static Vector2D flipHorizontally(Vector2D vector2D) {
        return new Vector2D(new double[] {-vector2D.getX(), vector2D.getY()});
    }

    public static Rotation2D flipHorizontally(Rotation2D rotation2D) {
        return new Rotation2D(AngleUtils.simplifyAngle(rotation2D.getRadian() + Math.PI));
    }
}
