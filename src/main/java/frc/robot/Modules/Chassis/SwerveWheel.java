package frc.robot.Modules.Chassis;

import frc.robot.Drivers.Encoders.Encoder;
import frc.robot.Utils.*;
import frc.robot.Drivers.Motors.Motor;
import frc.robot.Utils.MathUtils.AngleUtils;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.MechanismControllers.EnhancedPIDController;

/**
 * the module that controls a single swerve wheel, enabled with vector calculations
 *
 * note that the speed control in this module is a simple feed-forward control
 * it should be enabled during auto, pilot decides whether to use speed control, or percent output
 * there is only a very simple integral calculation in the speed control system to estimate friction
 */
public class SwerveWheel extends SwerveWheelLogic {
    private double wheelEncoderValuePerMeter;

    private final Motor drivingMotor;
    private final Motor steerMotor;
    private final Encoder drivingEncoder;
    private final Encoder steerEncoder;

    private EnhancedPIDController steerPIDController;
    private final double motorEncoderBias;

    /* <-- configurations --> */
    private double steerWheelErrorTolerance;
    private double steerWheelErrorStartDecelerate;
    private double steerWheelMaximumPower;
    private double steerWheelMinimumPower;
    private double steerWheelFeedForwardTime;
    private double steerCorrectionPowerRateAtZeroWheelSpeed;

    public SwerveWheel(
            Motor drivingMotor, Motor steerMotor,
            Encoder drivingEncoder, Encoder steerEncoder,
            Vector2D wheelPositionVector,
            RobotConfigReader robotConfig,
            int swerveWheelID,
            double motorEncoderBias) {
        super(swerveWheelID, robotConfig, wheelPositionVector);
        this.drivingMotor = drivingMotor;
        this.steerMotor = steerMotor;
        super.motors.add(drivingMotor);
        super.motors.add(steerMotor);
        this.drivingEncoder = drivingEncoder;
        this.steerEncoder = steerEncoder;
        this.motorEncoderBias = AngleUtils.simplifyAngle(motorEncoderBias); // critical, the motor encoder bias provided in the config is to the front, this actual motor encoder bias needs to be to the left
    }

    @Override
    public void init() {
        /* calibrate the steer encoder */
        this.steerEncoder.setZeroPosition(motorEncoderBias);
    }

    @Override
    public void periodic(double dt) {
        super.periodic(dt);
        final double steerEncoderCurrentReading = getSteerHeading(),
                steerEncoderCurrentVelocity = steerEncoder.getEncoderVelocity();

        /**
         * update the desired heading of the steer wheel, given control module's
         * instructions
         */
        final double decidedTargetedHeading = decideModuleDrivingDirection(), actualSwervePosition;
        /* whenever the reversed way is closer to get to, reverse the motor */
        if (reverseWheel =
                Math.abs(AngleUtils.getActualDifference(steerEncoderCurrentReading, decidedTargetedHeading)) > Math
                .abs(AngleUtils.getActualDifference(
                        steerEncoderCurrentReading, AngleUtils.simplifyAngle(decidedTargetedHeading + Math.PI)))
        )
            actualSwervePosition = AngleUtils.simplifyAngle(decidedTargetedHeading + Math.PI);
        else
            actualSwervePosition = decidedTargetedHeading;

        /* pass the desired position to pid controller */
        this.steerPIDController.startNewTask(new EnhancedPIDController.Task(EnhancedPIDController.Task.TaskType.GO_TO_POSITION, actualSwervePosition));

        /** the correction speed obtained from pid controller */
        double correctionMotorSpeed = steerPIDController.getMotorPower(steerEncoderCurrentReading, steerEncoderCurrentVelocity, dt);

        EasyDataFlow.putNumber("chassis", "steer " + swerveWheelID + " position", steerEncoderCurrentReading);
        EasyDataFlow.putNumber("chassis", "steer " + swerveWheelID + " velocity", steerEncoderCurrentVelocity);
        EasyDataFlow.putNumber("chassis", "steer " + swerveWheelID + "target", actualSwervePosition);

        /* given the power ratio of the steer, pass the PID feedback to the motor */
        final double steerPowerRate = locked ? 1: getSteerPowerRate(targetedSpeed);
        steerMotor.setPower(correctionMotorSpeed * steerPowerRate, this);

        /* update the motor power of the driving motor */
        actualDriveMotorPower = targetedSpeed; // TODO: here, write a speed controller

        // if (reverseWheel) drivePower *= -1;
        actualDriveMotorPower *= Math.cos(AngleUtils.getActualDifference(steerEncoderCurrentReading, commandedHeading)); // use cos instead, so that it runs slower when not reached target
        if (locked) actualDriveMotorPower = 0;

        drivingMotor.setPower(actualDriveMotorPower, this);
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
        super.updateConfigs();
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

        final double wheelDiameter = robotConfig.getConfig("chassis/wheelDiameter"),
                meterPerWheelRevolution = wheelDiameter * Math.PI,
                wheelGearRatio = robotConfig.getConfig("chassis/wheelGearRatio"),
                encoderValuePerMotorRevolution  = robotConfig.getConfig("chassis/encoderValuePerMotorRevolution"),
                encoderValuePerWheelRevolution = encoderValuePerMotorRevolution * wheelGearRatio;
        this.wheelEncoderValuePerMeter = encoderValuePerWheelRevolution / meterPerWheelRevolution;
    }

    @Override
    public void onReset() {
        super.onReset();

        drivingMotor.gainOwnerShip(this);
        steerMotor.gainOwnerShip(this);

        updateConfigs();
    }

    @Override
    protected void onDisable() {
        drivingMotor.disableMotor(this);
        steerMotor.disableMotor(this);
    }

    @Override
    protected void onEnable() {
        steerMotor.setMotorZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE, this);
        drivingMotor.setMotorZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE, this);
    }

    /**
     * get the heading of the wheel
     * @return the current heading of the wheel, in radian, zero the right and positive is counter-clockwise
     */
    @Override
    public double getSteerHeading() {
        return AngleUtils.simplifyAngle(steerEncoder.getEncoderPosition());
    }

    /**
     * gets the wheel's velocity from the driving motor's encoder
     * @return the velocity, direction is erased using the reversibility of the wheel
     */
    private double getWheelVelocityWithoutDirection() {
//        return drivingEncoder.getEncoderVelocity()
//                * (reverseWheel? -1 : 1);
        double encoderVel = drivingEncoder.getEncoderVelocity();
        return encoderVel * (reverseWheel? -1 : 1);
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


    @Override
    public double getWheelDrivingEncoderValue(ChassisUnit unit) {
        return drivingEncoder.getEncoderPosition() * scaleToUnit(unit);
    }

    @Override
    public Vector2D getModuleVelocity2D(ChassisUnit unit) {
        return new Vector2D(getSteerHeading(), drivingEncoder.getEncoderVelocity())
                .multiplyBy(scaleToUnit(unit));
    }

    @Override
    public boolean[] getErrors() {
        return new boolean[] {
                !drivingMotor.isAlive(),
                !steerMotor.isAlive(),
                !steerEncoder.isEncoderAlive()
        };
    }
}