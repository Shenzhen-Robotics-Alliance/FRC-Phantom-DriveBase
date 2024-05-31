package frc.robot.Modules.Chassis;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Modules.PositionReader.RobotFieldPositionEstimator;
import frc.robot.Modules.RobotModuleBase;
import frc.robot.Utils.EasyDataFlow;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.RobotConfigReader;
import frc.robot.Utils.RobotModuleOperatorMarker;

public abstract class SwerveDriveChassisLogic extends RobotModuleBase {
    /** the current translational task */
    protected ChassisTaskTranslation translationalTask;
    /** the current rotational task  */
    protected ChassisTaskRotation rotationalTask;
    protected OrientationMode orientationMode;
    private boolean lowSpeedModeEnabled;


    /** the four wheels of the robot */
    protected final SwerveWheelLogic frontLeft, frontRight, backLeft, backRight;
    protected final SwerveWheelLogic[] swerveWheels;
    protected final RobotFieldPositionEstimator positionEstimator;
    protected final RobotConfigReader robotConfig;
    private Vector2D displayedDesiredPosition = null;
    private Rotation2D displayedDesiredRotation = null;
    protected SwerveDriveChassisLogic(SwerveWheelLogic frontLeft, SwerveWheelLogic frontRight, SwerveWheelLogic backLeft, SwerveWheelLogic backRight, RobotFieldPositionEstimator positionEstimator, RobotConfigReader robotConfig) {
        super("chassis");
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.swerveWheels = new SwerveWheelLogic[] {frontLeft, frontRight, backLeft, backRight};
        this.positionEstimator = positionEstimator;
        this.robotConfig = robotConfig;
    }

    public enum OrientationMode {
        FIELD,
        ROBOT
    }

    protected double
            wheelsPowerConstrain,
            rotationalSpeedMaxSacrifice,
            wheelsPowerConstrainAtLowSpeedMode,
            robotMaximumSpeed;
    @Override
    public void updateConfigs() {
        this.robotMaximumSpeed = robotConfig.getConfig("chassis/robotMaximumSpeed");
        this.wheelsPowerConstrain = robotConfig.getConfig("chassis/wheelsPowerConstrain");
        this.wheelsPowerConstrainAtLowSpeedMode = robotConfig.getConfig("chassis/wheelsPowerConstrainAtLowSpeedMode");
        this.rotationalSpeedMaxSacrifice = robotConfig.getConfig("chassis/rotationalSpeedMaxSacrifice");
    }

    @Override
    protected void periodic(double dt) {
        EasyDataFlow.putNumber("chassis", "translational task (x)", translationalTask.translationValue.getX());
        EasyDataFlow.putNumber("chassis", "translational task (y)", translationalTask.translationValue.getY());
        EasyDataFlow.putNumber("chassis", "rotational task", rotationalTask.rotationalValue);

        EasyDataFlow.putPosition("/chassis/desiredPosition", displayedDesiredPosition, displayedDesiredRotation);
        EasyDataFlow.putPosition("/chassis/actualPosition", positionEstimator.getRobotPosition2D(), positionEstimator.getRobotRotation2D());
        SmartDashboard.putNumber("imu yaw:", Math.toDegrees(positionEstimator.getRobotRotation()));

        // TODO: the two values are zero over here
        EasyDataFlow.putNumber("test", "front left decided drive power", frontLeft.getModuleDecidedDrivingPower());
        EasyDataFlow.putNumber("test", "front left decided drive direction", frontLeft.decideModuleDrivingDirection());

        EasyDataFlow.putSwerveState(
                "desired swerve state",
                frontLeft.getModuleDecidedDrivingPower() * robotMaximumSpeed,
                frontLeft.decideModuleDrivingDirection(),
                frontRight.getModuleDecidedDrivingPower() * robotMaximumSpeed,
                frontRight.decideModuleDrivingDirection(),
                backLeft.getModuleDecidedDrivingPower() * robotMaximumSpeed,
                backLeft.decideModuleDrivingDirection(),
                backRight.getModuleDecidedDrivingPower() * robotMaximumSpeed,
                backRight.decideModuleDrivingDirection(),
                positionEstimator.getRobotRotation2D()
        );
    }

    protected void driveWheelsSafeLogic(Vector2D translationalSpeed, double rotationalSpeed) {
        EasyDataFlow.putNumber("chassis", "driving swerve safely to translational speed (X)", translationalSpeed.getX());
        EasyDataFlow.putNumber("chassis", "driving swerve safely to translational speed (Y)", translationalSpeed.getY());
        EasyDataFlow.putNumber("chassis", "driving swerve safely to rotational speed", rotationalSpeed);

        final double wheelsPowerConstrain = lowSpeedModeEnabled ? this.wheelsPowerConstrainAtLowSpeedMode : this.wheelsPowerConstrain;
        double highestWheelSpeed = driveWheelsLogic(translationalSpeed, rotationalSpeed);
        // System.out.println("highest wheel speed:" + highestWheelSpeed);
        if (highestWheelSpeed <= wheelsPowerConstrain) return;
        /* if a wheel is asked to run higher than max power, we need to slow everything down to avoid tearing the robot apart */

        /* first we slow down the rotational part */
        final double rotationMinScale = (1-rotationalSpeedMaxSacrifice);

        // System.out.println("sacrificing rotational part by scale: " + Math.sqrt(rotationMinScale));
        rotationalSpeed *= Math.sqrt(rotationMinScale);
        highestWheelSpeed = driveWheelsLogic(translationalSpeed, rotationalSpeed);
        if (highestWheelSpeed <= wheelsPowerConstrain) {
            return;
        }

        /* then we slow it down to max rotationalSpeedMaxSacrifice */
        EasyDataFlow.putNumber("chassis", "highest wheel speed:", highestWheelSpeed);
        EasyDataFlow.putNumber("chassis", "sacrificing rotational part by scale: ", rotationMinScale);
        rotationalSpeed *= Math.sqrt(rotationMinScale);
        highestWheelSpeed = driveWheelsLogic(translationalSpeed, rotationalSpeed);
        if (highestWheelSpeed <= wheelsPowerConstrain) return;

        /* finally, we start scaling down the translational part */
        EasyDataFlow.putNumber("chassis", "highest wheel speed:", highestWheelSpeed);
        EasyDataFlow.putNumber("chassis", "scaling down translational speed by factor:", wheelsPowerConstrain/highestWheelSpeed);
        translationalSpeed = translationalSpeed.multiplyBy(wheelsPowerConstrain/highestWheelSpeed);
        rotationalSpeed *= wheelsPowerConstrain/highestWheelSpeed;
        driveWheelsLogic(translationalSpeed, rotationalSpeed);
    }

    /**
     * pass the robot motion params to each wheels
     * @return the highest drive speed among the four wheels
     * */
    protected double driveWheelsLogic(Vector2D translationalSpeed, double rotationalSpeed) {
        double highestWheelSpeed = 0;
        for (SwerveWheelLogic wheel : swerveWheels)
            highestWheelSpeed = Math.max(highestWheelSpeed,
                    wheel.drive(translationalSpeed, rotationalSpeed, this));
        return highestWheelSpeed;
    }

    @Override
    public void init() {

    }

    @Override
    public void onReset() {
        this.lowSpeedModeEnabled = false;
        /* reset and recover ownerships to the wheels */
        for (SwerveWheelLogic swerveWheel: swerveWheels) {
            swerveWheel.reset();
            swerveWheel.gainOwnerShip(this);
        }
        /* reset the position calculator */
        positionEstimator.reset();
        positionEstimator.gainOwnerShip(this);

        /* reset translational and rotation tasks */
        System.out.println("setting chassis tasks to stop...");
        this.translationalTask = new ChassisTaskTranslation(ChassisTaskTranslation.TaskType.SET_VELOCITY, new Vector2D());
        this.rotationalTask = new ChassisTaskRotation(ChassisTaskRotation.TaskType.SET_VELOCITY, 0);
    }

    protected Vector2D getPilotDesiredVelocityToRobot() {
        return orientationMode == OrientationMode.FIELD ?
                processOrientation(
                        translationalTask.translationValue.multiplyBy(RobotFieldPositionEstimator.toActualRobotRotation(RobotFieldPositionEstimator.pilotFacingBlue))
                )
                : translationalTask.translationValue;
    }
    protected Vector2D processOrientation(Vector2D desiredVelocity) {
        return desiredVelocity
                .multiplyBy(
                        new Rotation2D(positionEstimator.getRobotRotation()).getReversal()
                );
    }

    public abstract double getChassisHeading();
    public ChassisTaskTranslation getCurrentTranslationalTask() {
        return this.translationalTask;
    }

    public ChassisTaskRotation getCurrentRotationalTask() {
        return this.rotationalTask;
    }

    /**
     * set the translational task of the chassis
     * @param translationalTask the desired task for rotation
     * @param operator the module or service that is calling for the task
     * */
    public void setTranslationalTask(ChassisTaskTranslation translationalTask, RobotModuleOperatorMarker operator) {
        if (!isOwner(operator))
            return;
        if (translationalTask == null)
            throw new IllegalArgumentException("translational task cannot be null!!!");
        this.translationalTask = translationalTask;
        this.displayedDesiredPosition = switch (translationalTask.taskType) {
            case SET_VELOCITY -> null;
            case GO_TO_POSITION -> translationalTask.translationValue;
        };
    }

    /**
     * sets the rotational task of the chassis
     * @param rotationalTask the desired task for rotation
     * @param operator the module or service that is calling for the task
     */
    public void setRotationalTask(ChassisTaskRotation rotationalTask, RobotModuleOperatorMarker operator) {
        if (!isOwner(operator))
            return;
        if (rotationalTask == null)
            throw new IllegalArgumentException("rotational task cannot be null!!!");

        this.rotationalTask = rotationalTask;
        this.displayedDesiredRotation = switch (rotationalTask.taskType) {
            case SET_VELOCITY -> null;
            case FACE_DIRECTION -> new Rotation2D(rotationalTask.rotationalValue);
        };
    }

    public void setLowSpeedModeEnabled(boolean enabled, RobotModuleOperatorMarker operator) {
        if (isOwner(operator))
            this.lowSpeedModeEnabled = enabled;
    }

    public void setOrientationMode(OrientationMode mode, RobotModuleOperatorMarker operator) {
        if (!this.isOwner(operator))
            return;
        this.orientationMode = mode;
    }

    public void setChassisLocked(boolean locked, RobotModuleOperatorMarker operator) {
        if (!this.isOwner(operator))
            return;
        for (SwerveWheelLogic swerveWheel:swerveWheels)
            swerveWheel.setWheelLocked(locked, this);
    }

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
