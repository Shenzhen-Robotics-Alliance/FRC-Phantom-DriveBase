package frc.robot.Utils.MathUtils;

import frc.robot.Utils.RobotConfigReader;

public class BezierCurveScheduleGenerator {
    private final double maximumVelocity, maximumAcceleration, maximumAngularVelocity;
    public BezierCurveScheduleGenerator(RobotConfigReader robotConfig) {
        this.maximumVelocity = robotConfig.getConfig("auto/autoStageMaxVelocity");
        this.maximumAcceleration = robotConfig.getConfig("auto/autoStageMaxAcceleration");
        this.maximumAngularVelocity = Math.toRadians(robotConfig.getConfig("auto/autoStageMaxAngularVelocity"));
    }

    public BezierCurveSchedule generateTranslationalSchedule(Vector2D startingPoint, Vector2D endingPoint) {
        return generateTranslationalSchedule(new BezierCurve(startingPoint, endingPoint));
    }

    public BezierCurveSchedule generateTranslationalSchedule(Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint) {
        return generateTranslationalSchedule(new BezierCurve(startingPoint, midPoint, endingPoint));
    }

    public BezierCurveSchedule generateTranslationalSchedule(BezierCurve path) {
        return new BezierCurveSchedule(maximumAcceleration, maximumVelocity, path);
    }

    public double getTimeNeededToFinishRotationalSchedule(double startingRotation, double endingRotation) {
        return Math.abs(AngleUtils.getActualDifference(startingRotation, endingRotation)) / maximumAngularVelocity;
    }
}
