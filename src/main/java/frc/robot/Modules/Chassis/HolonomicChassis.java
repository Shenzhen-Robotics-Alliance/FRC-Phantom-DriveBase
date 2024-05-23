package frc.robot.Modules.Chassis;

import frc.robot.Modules.RobotModuleBase;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.RobotModuleOperatorMarker;

/**
 * TODO: methods declaration for a holonomic chassis, and implement it using swerve-based chassis
 */
public abstract class HolonomicChassis extends RobotModuleBase {
    protected HolonomicChassis() {
        super("chassis");
    }

    public enum OrientationMode {
        FIELD,
        ROBOT
    }

    public abstract double getChassisHeading();
    public abstract ChassisTaskTranslation getCurrentTranslationalTask();

    public abstract ChassisTaskRotation getCurrentRotationalTask();

    public abstract void setTranslationalTask(ChassisTaskTranslation translationalTask, RobotModuleOperatorMarker operator);
    public abstract void setRotationalTask(ChassisTaskRotation rotationalTask, RobotModuleOperatorMarker operator);
    public abstract void setChassisLocked(boolean locked, RobotModuleOperatorMarker operator);
    public abstract void setOrientationMode(OrientationMode mode, RobotModuleOperatorMarker operator);
    public abstract void setLowSpeedModeEnabled(boolean enabled, RobotModuleOperatorMarker operator);

    public abstract boolean isCurrentTranslationalTaskFinished();

    public abstract boolean isCurrentRotationalTaskFinished();

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
