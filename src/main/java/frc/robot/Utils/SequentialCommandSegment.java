package frc.robot.Utils;
import frc.robot.Utils.MathUtils.AngleUtils;
import frc.robot.Utils.MathUtils.BezierCurve;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.SpeedCurves;

/**
 *  Instead of using static paths and rotation goals, we make them dynamic.
 *  A path feeder and two rotation feeders are specified in the auto stage command segments.
 *  The robot decides, just before the command segment is executed, what path should be followed and what rotation should be faced
 * */
public class SequentialCommandSegment {
    public final BezierCurveFeeder chassisMovementPathFeeder;
    public final Runnable beginning, periodic, ending;
    public final IsCompleteChecker isCompleteChecker;
    public final InitiateCondition initiateCondition;
    public final RotationFeeder startingRotationFeeder, endingRotationFeeder;
    public final SpeedCurves.SpeedCurve speedCurve;
    public final double timeScale;
    public SequentialCommandSegment (InitiateCondition initiateCondition, BezierCurveFeeder pathFeeder, Runnable beginning, Runnable periodic, Runnable ending, IsCompleteChecker isCompleteChecker, RotationFeeder startingRotation, RotationFeeder endingRotation) {
        this(initiateCondition, pathFeeder, beginning, periodic, ending, isCompleteChecker, startingRotation, endingRotation, SpeedCurves.originalSpeed, 1);
    }

    public SequentialCommandSegment(InitiateCondition initiateCondition, BezierCurveFeeder pathFeeder, Runnable beginning, Runnable periodic, Runnable ending, IsCompleteChecker isCompleteChecker, RotationFeeder startingRotation, RotationFeeder endingRotation, SpeedCurves.SpeedCurve speedCurve, double timeScale) {
        this.chassisMovementPathFeeder = pathFeeder;

        this.beginning = beginning;
        this.periodic = periodic;
        this.ending = ending;

        this.isCompleteChecker = isCompleteChecker;
        this.initiateCondition = initiateCondition;

        this.startingRotationFeeder = startingRotation;
        this.endingRotationFeeder = endingRotation;
        this.speedCurve = speedCurve;
        this.timeScale = timeScale;
    }

    public interface IsCompleteChecker {boolean isComplete();}
    public interface InitiateCondition {boolean initiateOrSkip();}
    public interface BezierCurveFeeder {BezierCurve getBezierCurve();}
    public interface RotationFeeder {Rotation2D getRotation();}

    /**
     * draw an instance of static command segment, using the feeders provided
     * this should be called right before the segment starts
     */
    public StaticSequentialCommandSegment embodyCurrentCommandSegment() {
        return new StaticSequentialCommandSegment(
                chassisMovementPathFeeder.getBezierCurve(),
                beginning,periodic,ending,initiateCondition, isCompleteChecker,
                startingRotationFeeder.getRotation(),endingRotationFeeder.getRotation(),
                this.speedCurve, this.timeScale
        );
    }
    /**
     * the static command segment generated from the feeders at the moment right before this segment start
     * */
    public static final class StaticSequentialCommandSegment {
        public final BezierCurve chassisMovementPath;
        public final Runnable beginning, periodic, ending;
        public final InitiateCondition initiateCondition;
        public final IsCompleteChecker isCompleteChecker;
        public final Rotation2D startingRotation, endingRotation;
        public final SpeedCurves.SpeedCurve speedCurve;
        public final double timeScale;
        public StaticSequentialCommandSegment(BezierCurve chassisMovementPath, Runnable beginning, Runnable periodic, Runnable ending, InitiateCondition initiateCondition, IsCompleteChecker isCompleteChecker, Rotation2D startingRotation, Rotation2D endingRotation, SpeedCurves.SpeedCurve speedCurve, double timeScale) {
            this.chassisMovementPath = chassisMovementPath;
            this.beginning = beginning;
            this.periodic = periodic;
            this.ending = ending;
            this.initiateCondition = initiateCondition;
            this.isCompleteChecker = isCompleteChecker;
            this.startingRotation = startingRotation;
            this.endingRotation = endingRotation;
            this.speedCurve = speedCurve;
            this.timeScale = timeScale;
        }

        public double getCurrentRotationWithLERP(double t) {
            if (startingRotation == null || endingRotation == null)
                throw new IllegalStateException("cannot obtain current rotation when the starting or ending rotation is not specified");
            t = speedCurve.getScaledT(t) * timeScale;
            if (t<0) t=0;
            else if (t>1) t=1;
            return AngleUtils.simplifyAngle(startingRotation.getRadian() + AngleUtils.getActualDifference(startingRotation.getRadian(), endingRotation.getRadian())*t);
        }
    }
}
