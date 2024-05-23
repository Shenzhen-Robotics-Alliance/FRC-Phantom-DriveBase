package frc.robot.Utils.MechanismControllers;

import frc.robot.Drivers.Encoders.Encoder;
import frc.robot.Drivers.Motors.Motor;
import frc.robot.Drivers.Motors.MotorWithLimitSwitch;
import frc.robot.Modules.RobotModuleBase;
import frc.robot.Utils.MathUtils.AngleUtils;

public class EncoderMotorMechanism implements Encoder, Motor {
    private final Encoder encoder;
    private final MotorWithLimitSwitch motor;
    private MechanismController controller = null;

    public EncoderMotorMechanism(Encoder encoder, Motor motor) {
        this.encoder = encoder;
        this.motor = new MotorWithLimitSwitch(motor);
    }

    @Override
    public void setPower(double power, RobotModuleBase operatorModule) {
        motor.setPower(power, operatorModule);
    }

    @Override
    public double getCurrentPower() {
        return motor.getCurrentPower();
    }

    @Override
    public void setMotorZeroPowerBehavior(ZeroPowerBehavior behavior, RobotModuleBase operatorModule) {
        motor.setMotorZeroPowerBehavior(behavior, operatorModule);
    }

    @Override
    public void disableMotor(RobotModuleBase operatorModule) {
        motor.disableMotor(operatorModule);
    }

    @Override
    public void lockMotor(RobotModuleBase operatorModule) {
        motor.lockMotor(operatorModule);
    }

    @Override
    public void setZeroPosition(double zeroPosition) {
        encoder.setZeroPosition(zeroPosition);
    }

    @Override
    public double getEncoderPosition() {
        return encoder.getEncoderPosition();
    }

    @Override
    public double getEncoderVelocity() {
        return encoder.getEncoderVelocity();
    }

    @Override
    public double getRawEncoderReading() {
        return encoder.getRawEncoderReading();
    }

    @Override
    public void gainOwnerShip(RobotModuleBase ownerModule) {
        motor.gainOwnerShip(ownerModule);
    }

    @Override
    public void onDestroy() {
        motor.onDestroy();
    }

    @Override
    public int getPortID() {
        return motor.getPortID();
    }

    public void setController(MechanismController controller) {
        this.controller = controller;
    }

    public double updateWithController(RobotModuleBase operatorModule) {
        if (controller == null) {
            motor.disableMotor(operatorModule);
            return 0;
        }
        final double power = controller.getMotorPower(encoder.getEncoderVelocity(), encoder.getEncoderPosition());
        setPower(power, operatorModule);
        return power;
    }

    /**
     * if limit is applied to the encoder, the encoder must be in absolute position and must be in radian
     * */
    public void setSoftEncoderLimit(double lowerLimit, double upperLimit) {
        motor.setPositiveDirectionLimitSwitch(() -> AngleUtils.getActualDifference(0, encoder.getEncoderPosition()) >= upperLimit);
        motor.setNegativeDirectionLimitSwitch(() -> AngleUtils.getActualDifference(0, encoder.getEncoderPosition()) <= lowerLimit);
    }
}
