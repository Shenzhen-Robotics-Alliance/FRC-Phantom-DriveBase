package frc.robot.Modules.Chassis;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drivers.Encoders.Encoder;
import frc.robot.Modules.RobotModuleBase;
import frc.robot.Utils.*;
import frc.robot.Drivers.Motors.Motor;
import frc.robot.Utils.MathUtils.AngleUtils;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Transformation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.MechanismControllers.EnhancedPIDController;
import frc.robot.Utils.MechanismControllers.SimpleFeedForwardSpeedController;

/**
 * the module that controls a single swerve wheel, enabled with vector calculations
 *
 * note that the speed control in this module is a simple feed-forward control
 * it should be enabled during auto, pilot decides whether to use speed control, or percent output
 * there is only a very simple integral calculation in the speed control system to estimate friction
 */
public class SwerveWheel extends RobotModuleBase {
    /** the encoder velocity obtained from the driving motor's encoder when functioning at full speed */
    private SimpleFeedForwardSpeedController wheelSpeedController;
    private double maxDrivingEncoderVelocity;
    private boolean driveSpeedControlEnabled;
    /** the friction of the driving wheel estimated by an integral algorithm, in percent motor power */
    private double estimatedFrictionPower;
    private double lastRecordedDriveSpeed;

    private double wheelEncoderValuePerMeter;

    private final int swerveWheelID;
    private final Vector2D wheelPositionVector;
    private final Vector2D rotationDirectionVector;
    private final Motor drivingMotor;
    private final Motor steerMotor;
    private final Encoder drivingEncoder;
    private final Encoder steerEncoder;
    private final RobotConfigReader robotConfig;

    private EnhancedPIDController steerPIDController;
    private final double motorEncoderBias;

    /* <-- configurations --> */
    /**
     * the maximum amount of time unused, after which the wheel will just go back to
     * default position
     */
    private double maxUnusedTime;
    /** the default position of the wheel */
    private double defaultPosition;
    /**
     * the slowest speed to be thought as being used, anything lower than this is
     * called "unused"
     */
    private double lowestUsageSpeed;
    private double steerWheelErrorTolerance;
    private double steerWheelErrorStartDecelerate;
    private double steerWheelMaximumPower;
    private double steerWheelMinimumPower;
    private double steerWheelFeedForwardTime;
    private double steerCorrectionPowerRateAtZeroWheelSpeed;
    private double steerCorrectionPowerFullWheelSpeed;

    /* <--variables--> */

    /** the desired position to drive */
    private double commandedHeading = 0;
    /** the set motor speed to drive */
    private double targetedSpeed = 0;
    /** the amount of time since last operation */
    private Timer lastOperationTimer = new Timer();
    /** whether to go to default position, or stick the pilot's order */
    private boolean stayInDefaultPosition = true; // make it true so it turns to front during startup

    /** whether this steer wheel is disabled and not allowed to move */
    private boolean disabled = false;

    /**
     * whether to reverse the wheel
     * when the wheel is reversed, the steer turns 180-degree to the desired heading
     * and the motor turns reversely
     * we do it whenever this reduces the distance for the wheel to turn
     */
    private boolean reverseWheel;
    private boolean locked;

    public SwerveWheel(
            Motor drivingMotor, Motor steerMotor,
            Encoder drivingEncoder, Encoder steerEncoder,
            Vector2D wheelPositionVector,
            RobotConfigReader robotConfig,
            int swerveWheelID,
            double motorEncoderBias) {
        super("swerve module " + swerveWheelID);
        this.swerveWheelID = swerveWheelID;
        this.wheelPositionVector = wheelPositionVector;
        this.drivingMotor = drivingMotor;
        this.steerMotor = steerMotor;
        super.motors.add(drivingMotor);
        super.motors.add(steerMotor);
        this.drivingEncoder = drivingEncoder;
        this.steerEncoder = steerEncoder;
        this.robotConfig = robotConfig;
        this.motorEncoderBias = AngleUtils.simplifyAngle(motorEncoderBias); // critical, the motor encoder bias provided in the config is to the front, this actual motor encoder bias needs to be to the left

        /*
         * calculate the direction of rotation motion of the robot in reference to the
         * wheel
         */
        Transformation2D rotate90DegCounterWiseTransformation = new Rotation2D(Math.toRadians(90));
        final Vector2D rotationDirectionVectorRaw = wheelPositionVector.multiplyBy(rotate90DegCounterWiseTransformation);
        this.rotationDirectionVector = new Vector2D(rotationDirectionVectorRaw.getHeading(), 0.8); // the rotation vector always have magnitude 1

    }

    @Override
    public void init() {
        /* calibrate the steer encoder */
        this.steerEncoder.setZeroPosition(motorEncoderBias);
        onReset();
    }

    @Override
    public void periodic(double dt) {
        steerMotor.setMotorZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE, this);
        drivingMotor.setMotorZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE, this);

        final double steerEncoderCurrentReading = getSteerHeading(),
                steerEncoderCurrentVelocity = getSteerVelocity();

        /** whether the robot is asked to move */
        boolean robotRequiredToMove = Math.abs(targetedSpeed) > lowestUsageSpeed;
        /* if there is an action */
        if (robotRequiredToMove) {
            lastOperationTimer.reset(); // reset the timer for last operation
            stayInDefaultPosition = false;
        }

        /* if the wheel is left unused for too long */
        if (lastOperationTimer.hasElapsed(maxUnusedTime))
            stayInDefaultPosition = true;

        /** the actual targeted steer heading decided by the system */
        double decidedTargetedHeading = stayInDefaultPosition ? defaultPosition : commandedHeading;

        /* proceed x-formation if required to lock */
        decidedTargetedHeading = locked ? getWheelInstalledLocationVector().getHeading(): decidedTargetedHeading;

        /**
         * update the desired heading of the steer wheel, given control module's
         * instructions
         */
        double finalTargetedHeading = decidedTargetedHeading;
        reverseWheel = false;
        /* whenever the reversed way is closer to get to, reverse the motor */
        if (Math.abs(AngleUtils.getActualDifference(steerEncoderCurrentReading, decidedTargetedHeading)) > Math
                .abs(AngleUtils.getActualDifference(
                        steerEncoderCurrentReading, AngleUtils.simplifyAngle(decidedTargetedHeading + Math.PI)))
        ) {
            reverseWheel = true;
            finalTargetedHeading = AngleUtils.simplifyAngle(finalTargetedHeading + Math.PI);
        }

        /* pass the desired position to pid controller */
        this.steerPIDController.startNewTask(new EnhancedPIDController.Task(EnhancedPIDController.Task.TaskType.GO_TO_POSITION, finalTargetedHeading));

        /** the correction speed obtained from pid controller */
        double correctionMotorSpeed = steerPIDController.getMotorPower(steerEncoderCurrentReading, steerEncoderCurrentVelocity, dt);

        EasyShuffleBoard.putNumber("chassis", "steer " + swerveWheelID + " position", steerEncoderCurrentReading);
        EasyShuffleBoard.putNumber("chassis", "steer " + swerveWheelID + " velocity", steerEncoderCurrentVelocity);
        EasyShuffleBoard.putNumber("chassis", "steer " + swerveWheelID + "target", finalTargetedHeading);

        /* given the power ratio of the steer, pass the PID feedback to the motor */
        final double steerPowerRate = locked ? 1: getSteerPowerRate(targetedSpeed);
        steerMotor.setPower(correctionMotorSpeed * steerPowerRate, this);

        /* update the motor power of the driving motor */
        double drivePower = targetedSpeed;
        if (driveSpeedControlEnabled)
            drivePower = wheelSpeedController.getSpeedControlPower(getWheelVelocityWithoutDirection() / this.maxDrivingEncoderVelocity, targetedSpeed);
//            drivePower = getDrivePower(getWheelVelocityWithoutDirection() / this.maxDrivingEncoderVelocity, targetedSpeed);

        // if (reverseWheel) drivePower *= -1;
        drivePower *= Math.cos(AngleUtils.getActualDifference(steerEncoderCurrentReading, commandedHeading)); // use cos instead, so that it runs slower when not reached target
        if (locked) drivePower = 0;

        drivingMotor.setPower(drivePower, this);
    }

    /**
     * with the current wheel speed, find the power rate for steers
     * the steer acts more rapidly as wheel speed is faster
     * @return the power rate, from 0~1
     * */
    private double getSteerPowerRate(double targetedSpeed) {
        return Math.min(
                steerCorrectionPowerRateAtZeroWheelSpeed
                        + Math.abs(targetedSpeed) * (1 - steerCorrectionPowerRateAtZeroWheelSpeed)
                , 1); // do not let it get over 1
    }

    @Override
    public void updateConfigs() {
        this.maxUnusedTime = robotConfig.getConfig("chassis/maxUnusedTime");
        this.defaultPosition = Math.toRadians(robotConfig.getConfig("chassis/defaultPosition"));
        this.lowestUsageSpeed = robotConfig.getConfig("chassis/minUsageSpeed");
        this.steerWheelErrorTolerance = Math.toRadians(robotConfig.getConfig("chassis/steerWheelErrorTolerance"));
        this.steerWheelErrorStartDecelerate = Math.toRadians(robotConfig.getConfig("chassis/steerWheelErrorStartDecelerate"));
        this.steerWheelMaximumPower = robotConfig.getConfig("chassis/steerWheelMaximumPower");
        this.steerWheelMinimumPower = robotConfig.getConfig("chassis/steerWheelMinimumPower");
        this.steerWheelFeedForwardTime = robotConfig.getConfig("chassis/steerWheelFeedForwardTime");

        this.steerPIDController = new EnhancedPIDController(new EnhancedPIDController.StaticPIDProfile(
                Math.PI * 2,
                steerWheelMaximumPower,
                steerWheelMinimumPower,
                steerWheelErrorStartDecelerate,
                steerWheelErrorTolerance,
                steerWheelFeedForwardTime,
                0,
                0
        ));
        this.steerCorrectionPowerRateAtZeroWheelSpeed = robotConfig.getConfig("chassis/steerCorrectionPowerRateAtZeroWheelSpeed");
        this.steerCorrectionPowerFullWheelSpeed = robotConfig.getConfig("chassis/steerCorrectionPowerFullWheelSpeed");

        this.maxDrivingEncoderVelocity = robotConfig.getConfig("chassis/maxDrivingEncoderVelocity");
        final double driveWheelProportionGain = robotConfig.getConfig("chassis/driveWheelProportionGain"),
                driveWheelFeedForwardRate = robotConfig.getConfig("chassis/driveWheelFeedForwardRate"),
                driveWheelFrictionDefaultValue = robotConfig.getConfig("chassis/driveWheelFrictionDefaultValue"),
                driveWheelFrictionIntegrationWeight = robotConfig.getConfig("chassis/driveWheelFrictionIntegrationWeight"),
                driveWheelFeedForwardDelay = robotConfig.getConfig("chassis/driveWheelFeedForwardDelay") / 1000,
                driveWheelVelocityTolerance = robotConfig.getConfig("chassis/driveWheelVelocityTolerance");

        this.wheelSpeedController = new SimpleFeedForwardSpeedController(
                new SimpleFeedForwardSpeedController.SimpleFeedForwardControllerProfile(
                        driveWheelProportionGain, driveWheelFeedForwardRate, driveWheelFrictionDefaultValue, driveWheelFeedForwardDelay
                ));

        final double wheelDiameter = robotConfig.getConfig("chassis/wheelDiameter"),
                meterPerWheelRevolution = wheelDiameter * Math.PI,
                wheelGearRatio = robotConfig.getConfig("chassis/wheelGearRatio"),
                encoderValuePerMotorRevolution  = robotConfig.getConfig("chassis/encoderValuePerMotorRevolution"),
                encoderValuePerWheelRevolution = encoderValuePerMotorRevolution * wheelGearRatio;
        this.wheelEncoderValuePerMeter = encoderValuePerWheelRevolution / meterPerWheelRevolution;
    }

    @Override
    public void onReset() {
        lastOperationTimer.start();
        lastRecordedDriveSpeed = 0;
        this.targetedSpeed = 0;
        this.commandedHeading = defaultPosition;
        this.driveSpeedControlEnabled = false;
        drivingMotor.gainOwnerShip(this);
        steerMotor.gainOwnerShip(this);

        updateConfigs();
    }

    /**
     * set to enable or disable speed control,
     * if disabled, the chassis drives using percent output
     * if enabled, the chassis controls its speed to the percent
     * @param enabled true for enable, false for disable
     */
    public void setSpeedControl(boolean enabled) {
        this.driveSpeedControlEnabled = enabled;
    }

    /**
     * estimate the amount of power needed to maintain the desired drive speed,
     * using a simple feed-forward algorithm
     * @param currentWheelSpeedPercent the current wheel speed obtained from encoders, in percentage to full speed, positive is to the desired direction
     * @param desiredDriveSpeedPercent the desired wheel speed, in percentage to the full speed
     */
//    @Deprecated
//    private double getDrivePower(double currentWheelSpeedPercent, double desiredDriveSpeedPercent) {
//        SimpleFeedForwardSpeedController.SpeedControllerProfile speedControllerProfile = wheelSpeedController.getSpeedControllerProfile();
//        double acceleration = (currentWheelSpeedPercent - lastRecordedDriveSpeed) / dt.get();
//        double speedError = desiredDriveSpeedPercent - (currentWheelSpeedPercent + acceleration* speedControllerProfile.feedForwardDelay);
//        double correctionPower = speedError * speedControllerProfile.feedForwardGain;
//        if (desiredDriveSpeedPercent > lowestUsageSpeed) { // add integral only when it's in use
//            correctionPower += estimatedFrictionPower;
////            double speedErrorForEstimation = (Math.abs(speedError) > driveWheelVelocityTolerance) ?
////                    Math.copySign(Math.abs(speedError)-driveWheelVelocityTolerance, speedError) : 0;
//            double speedErrorForEstimation =
//                    Math.max(desiredDriveSpeedPercent - driveWheelVelocityTolerance * desiredDriveSpeedPercent ,0)
//                            - currentWheelSpeedPercent; // the speed error for estimation needs to subtract the tolerance out, or the friction term will just be always growing
//            estimatedFrictionPower += speedErrorForEstimation * dt.get() * driveWheelFrictionIntegrationWeight;
//            estimatedFrictionPower = Math.min(estimatedFrictionPower, 1); // cannot exceed 1, or the robot looses control
//        }
//
//        lastRecordedDriveSpeed = currentWheelSpeedPercent;
//        return correctionPower;
//    }


    /**
     * set the driving parameter of the current steer module, given the raw params
     * of the wheel, the wheel will drive exactly the way you tell it if you call
     * this method
     * 
     * @param heading              the direction to drive, in radian. 0 is to the
     *                             front and
     *                             positive is counter-clockwise
     * @param speed                the motor speed to drive, in percent output. from
     *                             0~1
     * @param operator the robot service that is sending command to the
     *                             module, use this
     * @return the set motor speed
     */
    public double drive(double heading, double speed, RobotModuleOperatorMarker operator) {
        if (!isOwner(operator))
            return 0;
        final boolean inUse = speed > lowestUsageSpeed;
        this.commandedHeading = inUse ? heading : commandedHeading;
        this.targetedSpeed = inUse ? speed : 0;
        return speed;
    }

    /**
     * set the driving parameter of the current steer wheel, given a 2d vector with
     * x and y sitting in between0~1, representing the desired motion of the current
     * wheel
     * 
     * @param desiredWheelMotion   a 2d vector with x and y sitting in the bound
     *                             0~1, representing the motion of the robot
     * @param operator the robot service that is sending the commands,
     *                             use "this"
     * @return the magnitude of the given drive vector
     */
    public double drive(Vector2D desiredWheelMotion, RobotModuleOperatorMarker operator) {
        return drive(desiredWheelMotion.getHeading(), desiredWheelMotion.getMagnitude(), operator);
    }

    /**
     * set the motion of the wheel, given the robot's desired motion and this
     * function will process them automatically
     * 
     * @param desiredRobotMotion        a 2d vector with x and y in the range of
     *                                  0~1, representing the desired motion of the
     *                                  actual robot
     * @param desiredRobotRotationSpeed the desired angular speed of the robot,
     *                                  positive is counter-clockwise as in geometry
     * @return the amount of motor power of the wheel needed to do the operation, from 0~1
     * note that some operations might make motors go over full power, this will tear the chassis apart, so in chassis module we need to avoid that
     */
    public double drive(Vector2D desiredRobotMotion, double desiredRobotRotationSpeed, RobotModuleOperatorMarker operator) {
        Vector2D rotationVector = rotationDirectionVector.multiplyBy(desiredRobotRotationSpeed);
        return drive(desiredRobotMotion.addBy(rotationVector), operator);
    }

    public void setWheelLocked(boolean locked, RobotModuleOperatorMarker operatorMarker) {
        if (!isOwner(operatorMarker))
            return;
        this.locked = locked;
    }

    @Override
    protected void onDisable() {
        drivingMotor.disableMotor(this);
        steerMotor.disableMotor(this);
    }

    /** gets the position of the wheel on the robot */
    public Vector2D getWheelInstalledLocationVector() {
        return wheelPositionVector;
    }

    /** gets the direction vector of rotation of this wheel, which 90-deg counter-clockwise to the wheel */
    public Vector2D getRotationDirectionVector() {
        return rotationDirectionVector;
    }

    /**
     * get the heading of the wheel
     * @return the current heading of the wheel, in radian, zero the right and positive is counter-clockwise
     */
    public double getSteerHeading() {
        return AngleUtils.simplifyAngle(steerEncoder.getEncoderPosition());
    }

    /**
     * gets the angular velocity of the steer
     * @return in radian per second, positive is counter-clockwise
     */
    public double getSteerVelocity() {
        return steerEncoder.getEncoderVelocity();
    }

    /**
     * gets the wheel's velocity from the driving motor's encoder
     * @return the velocity, direction is erased using the reversibility of the wheel
     */
    public double getWheelVelocityWithoutDirection() {
//        return drivingEncoder.getEncoderVelocity()
//                * (reverseWheel? -1 : 1);
        long t0 = System.currentTimeMillis();
        double encoderVel = drivingEncoder.getEncoderVelocity();
        return encoderVel * (reverseWheel? -1 : 1);
    }



    public double getWheelDrivingEncoderValue() {
        return getWheelDrivingEncoderValue(ChassisUnit.METER);
    }
    public double getWheelDrivingEncoderValue(ChassisUnit unit) {
        return drivingEncoder.getEncoderPosition() * scaleToUnit(unit);
    }

    public Vector2D getModuleVelocity2D() {
        return getModuleVelocity2D(ChassisUnit.METER); // default value
    }
    public Vector2D getModuleVelocity2D(ChassisUnit unit) {
        return new Vector2D(getSteerHeading(), drivingEncoder.getEncoderVelocity())
                .multiplyBy(scaleToUnit(unit));
    }

    /**
     * distance (in unit X) = k * encoderValue
     * @param unit the given unit
     * @return the scale k
     * */
    private double scaleToUnit(ChassisUnit unit) {
        switch (unit) {
            case CENTIMETER:
                return scaleToUnit(ChassisUnit.METER) * 100;
            case METER:
                return 1.0/wheelEncoderValuePerMeter;
            case ENCODER_VALUE:
                return 1;
            default:
                throw new IllegalArgumentException("Invalid ChassisUnit: " + unit);
        }
    }

    /**
     * gets the motor speed needed for the steer to try to maintain, according to a
     * PID-like algorithm
     * 
     * @param rotationalError the difference between the desired steer direction and
     *                        the current, in radian
     * @param steerVelocity   the velocity of the steer motor, in radian per second
     * @return the amount of percentage output that the steer motor needs to get to
     *         the desired direction
     * @deprecated moved to simple-feed-forward controller
     */
    @Deprecated
    private double getSteerMotorCorrectionSpeed(double rotationalError, double steerVelocity) {
        /*
         * predict, according to the current steer velocity, the future rotational error
         */
        /* (Derivative) */
        double predictedRotationalError = rotationalError - steerVelocity * steerWheelFeedForwardTime;

        /* ignore any error smaller than the tolerance */
        if (Math.abs(predictedRotationalError) < steerWheelErrorTolerance)
            return 0;

        /*
         * find, according to the predicted error, the amount of motor power needed to
         * correct it
         */
        /* (Proportional) */
        double motorCorrectionSpeed = predictedRotationalError * steerWheelMaximumPower
                / steerWheelErrorStartDecelerate;

        /*
         * if the correction speed is too small to move the wheels, make it the minimum
         * power
         */
        if (Math.abs(motorCorrectionSpeed) < steerWheelMinimumPower * steerWheelMaximumPower)
            motorCorrectionSpeed = Math.copySign(steerWheelMinimumPower, motorCorrectionSpeed);

        /* if the correction speed is too big, restrict it down to the maximum */
        if (Math.abs(motorCorrectionSpeed) > steerWheelMaximumPower)
            motorCorrectionSpeed = Math.copySign(steerWheelMaximumPower, motorCorrectionSpeed);

        return motorCorrectionSpeed;
    }
}