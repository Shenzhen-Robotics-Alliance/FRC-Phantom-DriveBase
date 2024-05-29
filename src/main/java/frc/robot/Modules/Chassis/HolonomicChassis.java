package frc.robot.Modules.Chassis;

import frc.robot.Modules.RobotModuleBase;
import frc.robot.Utils.EasyDataFlow;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.RobotModuleOperatorMarker;

/**
 * TODO: methods declaration for a holonomic chassis, and implement it using swerve-based chassis
 */
public abstract class HolonomicChassis extends RobotModuleBase {
    private Vector2D displayedDesiredPosition = null;
    private Rotation2D displayedDesiredRotation = null;
    protected HolonomicChassis() {
        super("chassis");
    }

    public enum OrientationMode {
        FIELD,
        ROBOT
    }

    @Override
    protected void periodic(double dt) {
        EasyDataFlow.putPosition("/chassis/desiredPosition", displayedDesiredPosition, displayedDesiredRotation);
    }
    public abstract double getChassisHeading();
    public abstract ChassisTaskTranslation getCurrentTranslationalTask();

    public abstract ChassisTaskRotation getCurrentRotationalTask();

    public void setTranslationalTask(ChassisTaskTranslation translationalTask, RobotModuleOperatorMarker operator) {
        if (!isOwner(operator))
            return;
        this.displayedDesiredPosition = switch (translationalTask.taskType) {
            case SET_VELOCITY -> null;
            case GO_TO_POSITION -> translationalTask.translationValue;
        };
    }
    public void setRotationalTask(ChassisTaskRotation rotationalTask, RobotModuleOperatorMarker operator) {
        if (!isOwner(operator))
            return;
        this.displayedDesiredRotation = switch (rotationalTask.taskType) {
            case SET_VELOCITY -> null;
            case FACE_DIRECTION -> new Rotation2D(rotationalTask.rotationalValue);
        };
    }
    public abstract void setChassisLocked(boolean locked, RobotModuleOperatorMarker operator);
    public abstract void setOrientationMode(OrientationMode mode, RobotModuleOperatorMarker operator);
    public abstract void setLowSpeedModeEnabled(boolean enabled, RobotModuleOperatorMarker operator);

    public abstract boolean isCurrentTranslationalTaskFinished();

    public abstract boolean isCurrentRotationalTaskFinished();

    public abstract void resetChassisPositionAndRotation();

    public static final class ChassisTaskTranslation {
        public enum TaskType {
            SET_VELOCITY,
            GO_TO_POSITION
        }
        public final TaskType taskType;
        public final Vector2D translationValue;

        public ChassisTaskTranslation(TaskType type, Vector2D value) {
            this.taskType = type;
            this.translationValue = value;
        }
    }

    public static final class ChassisTaskRotation {
        public enum TaskType {
            SET_VELOCITY,
            FACE_DIRECTION
        }
        public final TaskType taskType;
        public double rotationalValue;

        /** creates a rotational task
         * @param type choose from this.TaskType
         * @param value in radians, counter-clockwise is positive
         */
        public ChassisTaskRotation(TaskType type, double value) {
            this.taskType = type;
            this.rotationalValue = value;
        }
    }
}
