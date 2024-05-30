package frc.robot.Modules.Chassis;

import frc.robot.Modules.PositionReader.RobotFieldPositionEstimator;
import frc.robot.Utils.*;
import frc.robot.Utils.MathUtils.AngleUtils;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.MechanismControllers.EnhancedPIDController;

/**
 * the module that controls the chassis with its four swerves
 */
public class SwerveDriveChassis extends SwerveDriveChassisLogic {
    private final boolean useProfiledSpeedControl = true;
    private double speedControlAccelerateTime;
    private double robotSpeedActivateSpeedControl;
    /** the pid controller that controls the rotation of the robot when needed */
    private EnhancedPIDController goToRotationController;
    private ChassisPositionController chassisPIDController;

    private double positionDifferenceAsTaskFinished, rotationDifferenceAsTaskFinished;



    public SwerveDriveChassis(SwerveWheelLogic frontLeft, SwerveWheelLogic frontRight, SwerveWheelLogic backLeft, SwerveWheelLogic backRight, RobotFieldPositionEstimator positionEstimator, RobotConfigReader robotConfig) {

        super(frontLeft, frontRight, backLeft, backRight, positionEstimator, robotConfig);
    }

    @Override
    public void init() {

    }

    @Override
    protected void periodic(double dt) {
        EasyDataFlow.putSwerveState(
                "actual swerve state",
                frontLeft.getModuleVelocity2D(ChassisUnit.METER).getMagnitude(),
                frontLeft.getWheelDrivingEncoderValue(),
                frontRight.getModuleVelocity2D(ChassisUnit.METER).getMagnitude(),
                frontRight.getWheelDrivingEncoderValue(),
                backLeft.getModuleVelocity2D(ChassisUnit.METER).getMagnitude(),
                backLeft.getWheelDrivingEncoderValue(),
                backRight.getModuleVelocity2D(ChassisUnit.METER).getMagnitude(),
                backRight.getWheelDrivingEncoderValue(),
                positionEstimator.getRobotRotation2D()
        );

        Vector2D processedTranslationalSpeed = processTranslationalMotion(dt);
        double rotationalSpeed = processRotationalMotion(dt);

        driveWheelsSafeLogic(processedTranslationalSpeed, rotationalSpeed);

        super.periodic(dt);
    }

    @Override
    public void updateConfigs() {
        super.updateConfigs();
        this.speedControlAccelerateTime = robotConfig.getConfig("chassis/timeNeededToFullyAccelerate");
        this.robotSpeedActivateSpeedControl = robotConfig.getConfig("chassis/robotSpeedActivateSpeedControl");
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
        super.onReset();

        this.decidedVelocity = new Vector2D();
    }

    private Vector2D processTranslationalMotion(double dt) {
        return switch (translationalTask.taskType) {
            case SET_VELOCITY -> processTranslationalVelocityControl(getPilotDesiredVelocityToRobot(), dt
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
    public double getChassisHeading() {
        return positionEstimator.getRobotRotation();
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
