package frc.robot.Modules.Chassis;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Modules.PositionReader.RobotFieldPositionEstimator;
import frc.robot.Modules.RobotModuleBase;
import frc.robot.Utils.*;
import frc.robot.Utils.MathUtils.AngleUtils;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.MechanismControllers.EnhancedPIDController;

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

    /** the pid controller that controls the rotation of the robot when needed */
    private EnhancedPIDController goToRotationController;
    private ChassisPositionController chassisPIDController;
    private double positionDifferenceAsTaskFinished, rotationDifferenceAsTaskFinished;
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


        double robotRotationalErrorTolerance = Math.toRadians(robotConfig.getConfig("chassis/robotRotationalErrorTolerance"));
        this.rotationDifferenceAsTaskFinished = Math.toRadians(robotConfig.getConfig("chassis/rotationalErrorAsCommandFinished"));
        double robotRotationalErrorStartDecelerate = Math.toRadians(robotConfig.getConfig("chassis/robotRotationalErrorStartDecelerate"));
        double robotRotationMaximumCorrectionPower = robotConfig.getConfig("chassis/robotRotationMaximumCorrectionPower");
        double robotRotationMinimumCorrectionPower = robotConfig.getConfig("chassis/robotRotationMinimumCorrectionPower");
        double robotRotationFeedForwardTime = robotConfig.getConfig("chassis/robotRotationFeedForwardTime");

        double robotPositionErrorTolerance = robotConfig.getConfig("chassis/robotPositionErrorTolerance");
        this.positionDifferenceAsTaskFinished = robotConfig.getConfig("chassis/translationalErrorAsCommandFinished");
        double robotPositionErrorStartDecelerate = robotConfig.getConfig("chassis/robotPositionErrorStartDecelerate");
        double robotPositionMaximumCorrectionPower = robotConfig.getConfig("chassis/robotPositionMaximumCorrectionPower");
        double robotPositionMinimumCorrectionPower = robotConfig.getConfig("chassis/robotPositionMinimumCorrectionPower");
        double robotPositionFeedForwardTime = robotConfig.getConfig("chassis/robotPositionFeedForwardTime");

        this.goToRotationController = new EnhancedPIDController(new EnhancedPIDController.StaticPIDProfile(
                Math.PI * 2,
                robotRotationMaximumCorrectionPower,
                robotRotationMinimumCorrectionPower,
                robotRotationalErrorStartDecelerate,
                robotRotationalErrorTolerance,
                robotRotationFeedForwardTime,
                0,
                0
        ));

        final ChassisPositionController.ChassisPIDConfig chassisPIDConfig = new ChassisPositionController.ChassisPIDConfig(
                robotPositionMaximumCorrectionPower,
                robotPositionMinimumCorrectionPower,
                robotPositionErrorStartDecelerate,
                robotPositionErrorTolerance,
                robotPositionFeedForwardTime
        );

        chassisPIDController = new ChassisPositionController(chassisPIDConfig);

        this.speedControlAccelerateTime = robotConfig.getConfig("chassis/timeNeededToFullyAccelerate");
        this.robotSpeedActivateSpeedControl = robotConfig.getConfig("chassis/robotSpeedActivateSpeedControl");
        this.ignoredAccelerateTime = robotConfig.getConfig("chassis/ignoredAccelerateTime");
    }

    @Override
    protected void periodic(double dt) {
        EasyDataFlow.putNumber("chassis", "translational task (x)", translationalTask.translationValue.getX());
        EasyDataFlow.putNumber("chassis", "translational task (y)", translationalTask.translationValue.getY());
        EasyDataFlow.putNumber("chassis", "rotational task", rotationalTask.rotationalValue);

        EasyDataFlow.putPosition("chassis/desiredPosition", displayedDesiredPosition, displayedDesiredRotation);
        EasyDataFlow.putPosition("chassis/actualPosition", positionEstimator.getRobotPosition2D(), positionEstimator.getRobotRotation2D());
        SmartDashboard.putNumber("imu yaw:", Math.toDegrees(positionEstimator.getRobotRotation()));

        displayDesiredSwerveStates();
    }

    protected Vector2D calculateTranslationalPowerToRobot(double dt) {
        return switch (translationalTask.taskType) {
            case SET_VELOCITY -> processTranslationalVelocityControl(getPilotDesiredVelocityToRobot(), dt);
            case GO_TO_POSITION -> processOrientation(
                    processTranslationalPositionControl(this.translationalTask.translationValue)
            );
        };
    }

    private double speedControlAccelerateTime, robotSpeedActivateSpeedControl;
    private Vector2D decidedVelocity;
    /** if the chassis can accelerate to the targeted velocity within this amount time, it just jumps to the target */
    private double ignoredAccelerateTime;
    /**
     * process the desired velocity into actual respond velocity using profiled speed-control
     * @param desiredVelocity the desired velocity, in relationship to the robot itself and in PERCENT OUT OF FULL SPEED
     * @return the amount
     * */
    protected Vector2D processTranslationalVelocityControl(Vector2D desiredVelocity, double dt) {
        if (Math.max(desiredVelocity.getMagnitude(), positionEstimator.getRobotVelocity2DToField().getMagnitude()/robotMaximumSpeed)
                < robotSpeedActivateSpeedControl) // if the desired or current velocity is smaller than the activation speed
            return desiredVelocity; // disable velocity control

        final double maxAcceleration = 1.0 / speedControlAccelerateTime;
        Vector2D velocityDifference = desiredVelocity.addBy(
                decidedVelocity.multiplyBy(-1)
        );

        EasyDataFlow.putNumber("chassis", "vel ctrl decided", decidedVelocity.getMagnitude());
        Vector2D step = new Vector2D(velocityDifference.getHeading(),
                Math.min(dt * maxAcceleration, velocityDifference.getMagnitude())
        );
        this.decidedVelocity = decidedVelocity.addBy(step);
        if (velocityDifference.getMagnitude() < maxAcceleration * ignoredAccelerateTime)
            return desiredVelocity;
        return decidedVelocity;
    }

    /**
     * gets the correction power to move to a given position
     * @param desiredPosition the desired position, field-orientated
     * @return the amount of chassis speed the robot needs at the current time, field-orientated
     */
    protected Vector2D processTranslationalPositionControl(Vector2D desiredPosition) {
        final Vector2D chassisPosition2D = positionEstimator.getRobotPosition2D(),
                chassisVelocity2D = positionEstimator.getRobotVelocity2DToField();

        chassisPIDController.setDesiredPosition(desiredPosition);
        return chassisPIDController.getCorrectionPower(chassisPosition2D, chassisVelocity2D);
    }

    protected double calculateRotationalPower(double dt) {
        return switch (rotationalTask.taskType) {
            case SET_VELOCITY -> this.rotationalTask.rotationalValue;
            case FACE_DIRECTION -> {
                goToRotationController.startNewTask(new EnhancedPIDController.Task(
                        EnhancedPIDController.Task.TaskType.GO_TO_POSITION,
                        this.rotationalTask.rotationalValue
                ));
                yield goToRotationController.getMotorPower(positionEstimator.getRobotRotation(), positionEstimator.getRobotRotationalVelocity(), dt);
            }
        };
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

        this.decidedVelocity = new Vector2D();
    }

    private void displayDesiredSwerveStates() {
        EasyDataFlow.putSwerveState(
                "chassis/desired swerve state",
                frontLeft.getDesiredSwerveState()[0],
                frontLeft.getDesiredSwerveState()[1],
                frontRight.getDesiredSwerveState()[0],
                frontRight.getDesiredSwerveState()[1],
                backLeft.getDesiredSwerveState()[0],
                backLeft.getDesiredSwerveState()[1],
                backRight.getDesiredSwerveState()[0],
                backRight.getDesiredSwerveState()[1],
                positionEstimator.getRobotRotation2D()
        );
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

    public boolean isCurrentTranslationalTaskFinished() {
        System.out.println("<-- waiting for chassis to finish movement -->");
        System.out.println("Task Type: " + translationalTask.taskType);
        System.out.println("Translational Error: " + Vector2D.displacementToTarget(positionEstimator.getRobotPosition2D(), translationalTask.translationValue));
        System.out.println("Tolerance: " + positionDifferenceAsTaskFinished);
        return switch (translationalTask.taskType) {
            case SET_VELOCITY ->
                    translationalTask.translationValue.getMagnitude() == 0;
            case GO_TO_POSITION ->
                    Vector2D.displacementToTarget(positionEstimator.getRobotPosition2D(), translationalTask.translationValue).getMagnitude() < positionDifferenceAsTaskFinished;
        };
    }

    public boolean isCurrentRotationalTaskFinished() {
        System.out.println("rotational error: " + Math.toDegrees(Math.abs(AngleUtils.getActualDifference(getChassisHeading(), rotationalTask.rotationalValue))) + ", tolerance: " + Math.toDegrees(rotationDifferenceAsTaskFinished));
        return switch (rotationalTask.taskType) {
            case SET_VELOCITY -> rotationalTask.rotationalValue == 0;
            case FACE_DIRECTION -> Math.abs(AngleUtils.getActualDifference(getChassisHeading(), rotationalTask.rotationalValue)) < rotationDifferenceAsTaskFinished;
        };
    }

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
    protected final Alert imuError = new Alert("Fatal: IMU Module disconnected", Alert.AlertType.ERROR),
            frontLeftModuleError = new Alert("Fatal: Front-Left Module Error", Alert.AlertType.ERROR),
            frontRightModuleError = new Alert("Fatal: Front-Right Module Error", Alert.AlertType.ERROR),
            backLeftModuleError = new Alert("Fatal: Back-Left Module Error", Alert.AlertType.ERROR),
            backRightModuleError = new Alert("Fatal: Back-Right Module Error", Alert.AlertType.ERROR);
}
