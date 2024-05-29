package frc.robot.Modules.Chassis;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Modules.PositionReader.RobotFieldPositionEstimator;
import frc.robot.Utils.*;
import frc.robot.Utils.MathUtils.AngleUtils;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.MechanismControllers.EnhancedPIDController;

/**
 * the module that controls the chassis with its four swerves
 */
public class SwerveDriveChassis extends SwerveDriveChassisLogic {
    private final boolean useProfiledSpeedControl = true;
    private double robotMaximumSpeed;
    private double timeNeededToFullyAccelerate;
    private double robotSpeedActivateSpeedControl;
    private double wheelsPowerConstrain;
    private double rotationalSpeedMaxSacrifice;
    /** the pid controller that controls the rotation of the robot when needed */
    private EnhancedPIDController goToRotationController;
    private ChassisPositionController chassisPIDController;

    private double positionDifferenceAsTaskFinished, rotationDifferenceAsTaskFinished, wheelsPowerConstrainAtLowSpeedMode;
    private boolean lowSpeedModeEnabled;



    /** the current translational task */
    private ChassisTaskTranslation translationalTask;
    /** the current rotational task  */
    private ChassisTaskRotation rotationalTask;

    private OrientationMode orientationMode;


    /** the four wheels of the robot */
    private final SwerveWheelLogic[] swerveWheels;
    public final RobotFieldPositionEstimator positionEstimator;
    private final RobotConfigReader robotConfig;
    public SwerveDriveChassis(SwerveWheelLogic[] swerveWheels, RobotConfigReader robotConfig, RobotFieldPositionEstimator positionEstimator) {
        this.swerveWheels = swerveWheels;
        this.positionEstimator = positionEstimator;
        this.robotConfig = robotConfig;
    }

    @Override
    public void init() {
        this.onReset();
    }

    @Override
    protected void periodic(double dt) {
        super.periodic(dt);

        EasyDataFlow.putNumber("chassis", "chassis task (x)", translationalTask.translationValue.getX());
        EasyDataFlow.putNumber("chassis", "chassis task (y)", translationalTask.translationValue.getY());
        Vector2D processedTranslationalSpeed = processTranslationalMotion(dt);
        double rotationalSpeed = processRotationalMotion(dt);


        SmartDashboard.putNumber("imu yaw:", Math.toDegrees(positionEstimator.getRobotRotation()));

        final double wheelsPowerConstrain = lowSpeedModeEnabled ? this.wheelsPowerConstrainAtLowSpeedMode : this.wheelsPowerConstrain;
        double highestWheelSpeed = driveWheels(processedTranslationalSpeed, rotationalSpeed);
        // System.out.println("highest wheel speed:" + highestWheelSpeed);
        if (highestWheelSpeed <= wheelsPowerConstrain) return;
        /* if a wheel is asked to run higher than max power, we need to slow everything down to avoid tearing the robot apart */

        /* first we slow down the rotational part */
        final double rotationMinScale = (1-rotationalSpeedMaxSacrifice);

        // System.out.println("sacrificing rotational part by scale: " + Math.sqrt(rotationMinScale));
        rotationalSpeed *= Math.sqrt(rotationMinScale);
        highestWheelSpeed = driveWheels(processedTranslationalSpeed, rotationalSpeed);
        if (highestWheelSpeed <= wheelsPowerConstrain) {
            return;
        }

        /* then we slow it down to max rotationalSpeedMaxSacrifice */
        EasyDataFlow.putNumber("chassis", "highest wheel speed:", highestWheelSpeed);
        EasyDataFlow.putNumber("chassis", "sacrificing rotational part by scale: ", rotationMinScale);
        rotationalSpeed *= Math.sqrt(rotationMinScale);
        highestWheelSpeed = driveWheels(processedTranslationalSpeed, rotationalSpeed);
        if (highestWheelSpeed <= wheelsPowerConstrain) return;

        /* finally, we start scaling down the translational part */
        EasyDataFlow.putNumber("chassis", "highest wheel speed:", highestWheelSpeed);
        EasyDataFlow.putNumber("chassis", "scaling down translational speed by factor:", wheelsPowerConstrain/highestWheelSpeed);
        processedTranslationalSpeed = processedTranslationalSpeed.multiplyBy(wheelsPowerConstrain/highestWheelSpeed);
        rotationalSpeed *= wheelsPowerConstrain/highestWheelSpeed;
        driveWheels(processedTranslationalSpeed, rotationalSpeed);
    }

    /**
     * pass the robot motion params to each wheels
     * @return the highest drive speed among the four wheels
     * */
    private double driveWheels(Vector2D translationalSpeed, double rotationalSpeed) {
        double highestWheelSpeed = 0;
        for (SwerveWheelLogic wheel : swerveWheels)
            highestWheelSpeed = Math.max(highestWheelSpeed,
                    wheel.drive(translationalSpeed, rotationalSpeed, this));
        return highestWheelSpeed;
    }

    @Override
    public void updateConfigs() {
        this.robotMaximumSpeed = robotConfig.getConfig("chassis/robotMaximumSpeed");
        this.timeNeededToFullyAccelerate = robotConfig.getConfig("chassis/timeNeededToFullyAccelerate");
        this.robotSpeedActivateSpeedControl = robotConfig.getConfig("chassis/robotSpeedActivateSpeedControl");
        this.wheelsPowerConstrain = robotConfig.getConfig("chassis/wheelsPowerConstrain");
        this.wheelsPowerConstrainAtLowSpeedMode = robotConfig.getConfig("chassis/wheelsPowerConstrainAtLowSpeedMode");
        this.rotationalSpeedMaxSacrifice = robotConfig.getConfig("chassis/rotationalSpeedMaxSacrifice");
        this.ignoredAccelerateTime = robotConfig.getConfig("chassis/ignoredAccelerateTime");

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
    }

    @Override
    public void onReset() {
        System.out.println("<-- chassis reset coming through --> ");
        /* reset and recover ownerships to the wheels */
        for (SwerveWheelLogic swerveWheel: swerveWheels) {
            swerveWheel.reset();
            swerveWheel.gainOwnerShip(this);
        }
        /* reset the position calculator */
        positionEstimator.reset();
        this.translationalTask = new ChassisTaskTranslation(ChassisTaskTranslation.TaskType.SET_VELOCITY, new Vector2D());
        this.rotationalTask = new ChassisTaskRotation(ChassisTaskRotation.TaskType.SET_VELOCITY, 0);

        this.decidedVelocity = new Vector2D();
        this.lowSpeedModeEnabled = false;
    }

    private Vector2D processTranslationalMotion(double dt) {
        return switch (translationalTask.taskType) {
            case SET_VELOCITY -> processTranslationalVelocityControl( // process the speed control after
                    orientationMode == OrientationMode.FIELD ?
                            processOrientation(
                                    translationalTask.translationValue.multiplyBy(RobotFieldPositionEstimator.toActualRobotRotation(RobotFieldPositionEstimator.pilotFacingBlue))
                            )
                            : translationalTask.translationValue, dt
            );
            case GO_TO_POSITION -> processOrientation(
                    processTranslationalPositionControl(this.translationalTask.translationValue)
            );
        };
    }

    private Vector2D decidedVelocity;
    /** if the chassis can accelerate to the targeted velocity within this amount time, it just jumps to the target */
    private double ignoredAccelerateTime;
    /**
     * process the desired velocity into actual respond velocity using profiled speed-control
     * @param desiredVelocity the desired velocity, in relationship to the robot itself and in PERCENT OUT OF FULL SPEED
     * @return the amount
     * */
    private Vector2D processTranslationalVelocityControl(Vector2D desiredVelocity, double dt) {
        if (!useProfiledSpeedControl
                || Math.max(desiredVelocity.getMagnitude(), positionEstimator.getRobotVelocity2D().getMagnitude()/robotMaximumSpeed)
                < robotSpeedActivateSpeedControl) // if the desired or current velocity is smaller than the activation speed
            return desiredVelocity; // disable velocity control

        final double maxAcceleration = 1.0 / timeNeededToFullyAccelerate;
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

    private Vector2D processOrientation(Vector2D desiredVelocity) {
        return desiredVelocity
                .multiplyBy(
                        new Rotation2D(positionEstimator.getRobotRotation()).getReversal()
                );
    }

    /**
     * gets the correction power to move to a given position
     * @param desiredPosition the desired position, field-orientated
     * @return the amount of chassis speed the robot needs at the current time, field-orientated
     */
    public Vector2D processTranslationalPositionControl(Vector2D desiredPosition) {
        final Vector2D chassisPosition2D = positionEstimator.getRobotPosition2D(),
                chassisVelocity2D = positionEstimator.getRobotVelocity2D();

        chassisPIDController.setDesiredPosition(desiredPosition);
        return chassisPIDController.getCorrectionPower(chassisPosition2D, chassisVelocity2D);
    }

    private double processRotationalMotion(double dt) {
        return switch (rotationalTask.taskType) {
            case SET_VELOCITY -> processRotationalVelocity(this.rotationalTask.rotationalValue);
            case FACE_DIRECTION -> getRotationalCorrectionSpeed(this.rotationalTask.rotationalValue, dt);
        };
    }

    private double processRotationalVelocity(double desiredRotationalVelocity) {
        // TODO write the profiled rotational velocity controller here
        return desiredRotationalVelocity;
    }

    /**
     * get the amount of chassis speed needed in order to maintain the desired rotation
     *
     * @param desiredRotation
     * @return
     */
    private double getRotationalCorrectionSpeed(double desiredRotation, double dt) {
        goToRotationController.startNewTask(new EnhancedPIDController.Task(
                EnhancedPIDController.Task.TaskType.GO_TO_POSITION,
                desiredRotation
        ));
        return goToRotationController.getMotorPower(AngleUtils.simplifyAngle(positionEstimator.getRobotRotation()), positionEstimator.getRobotRotationalVelocity(), dt);
    }

    @Override
    public void setLowSpeedModeEnabled(boolean enabled, RobotModuleOperatorMarker operator) {
        if (isOwner(operator))
            this.lowSpeedModeEnabled = enabled;
    }

    /**
     * set the translational task of the chassis
     * @param translationalTask the desired task for rotation
     * @param operator the module or service that is calling for the task
     * */
    @Override
    public void setTranslationalTask(ChassisTaskTranslation translationalTask, RobotModuleOperatorMarker operator) {
        super.setTranslationalTask(translationalTask, operator);
        if (!this.isOwner(operator))
                return;

        this.translationalTask = translationalTask;
    }

    @Override
    public void setOrientationMode(OrientationMode mode, RobotModuleOperatorMarker operator) {
        if (!this.isOwner(operator))
            return;
        this.orientationMode = mode;
    }

    /**
     * sets the rotational task of the chassis
     * @param rotationalTask the desired task for rotation
     * @param operator the module or service that is calling for the task
     */
    @Override
    public void setRotationalTask(ChassisTaskRotation rotationalTask, RobotModuleOperatorMarker operator) {
        super.setRotationalTask(rotationalTask, operator);
        if (!this.isOwner(operator))
            return;
        this.rotationalTask = rotationalTask;
    }

    @Override
    public void setChassisLocked(boolean locked, RobotModuleOperatorMarker operator) {
        if (!this.isOwner(operator))
            return;
        for (SwerveWheelLogic swerveWheel:swerveWheels)
            swerveWheel.setWheelLocked(locked, this);
    }

    @Override
    public double getChassisHeading() {
        return positionEstimator.getRobotRotation();
    }

    @Override
    public ChassisTaskTranslation getCurrentTranslationalTask() {
        return translationalTask;
    }

    @Override
    public ChassisTaskRotation getCurrentRotationalTask() {
        return rotationalTask;
    }

    @Override
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

    @Override
    public boolean isCurrentRotationalTaskFinished() {
        System.out.println("rotational error: " + Math.toDegrees(Math.abs(AngleUtils.getActualDifference(getChassisHeading(), rotationalTask.rotationalValue))) + ", tolerance: " + Math.toDegrees(rotationDifferenceAsTaskFinished));
        return switch (rotationalTask.taskType) {
            case SET_VELOCITY -> rotationalTask.rotationalValue == 0;
            case FACE_DIRECTION -> Math.abs(AngleUtils.getActualDifference(getChassisHeading(), rotationalTask.rotationalValue)) < rotationDifferenceAsTaskFinished;
        };
    }

    @Override
    public void resetChassisPositionAndRotation() {
        positionEstimator.reset();
    }
}
