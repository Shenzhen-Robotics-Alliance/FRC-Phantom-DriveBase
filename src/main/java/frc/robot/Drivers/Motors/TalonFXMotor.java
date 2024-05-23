package frc.robot.Drivers.Motors;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Drivers.Encoders.Encoder;
import frc.robot.Drivers.RobotDriverBase;
import frc.robot.Modules.RobotModuleBase;
import frc.robot.Utils.MechanismControllers.EncoderMotorMechanism;

public class TalonFXMotor extends RobotDriverBase implements Motor, Encoder {
    private final TalonFX talonFXInstance;
    private final int portID;
    /** encoder is built-in, so they reverse together */
    private double powerAndEncoderScaleFactor;
    private double currentPower = 0, zeroPosition = 0;
    private boolean enabled;
    private ZeroPowerBehavior zeroPowerBehavior;

    public TalonFXMotor(TalonFX talonFXInstance) {
        this(talonFXInstance, false);
    }

    public TalonFXMotor(TalonFX talonFXInstance, boolean reversed) {
        this.powerAndEncoderScaleFactor = reversed ? -1 : 1;
        this.talonFXInstance = talonFXInstance;
        talonFXInstance.getRotorPosition().setUpdateFrequency(100);
        talonFXInstance.getRotorVelocity().setUpdateFrequency(100);
        this.portID = talonFXInstance.getDeviceID();
        enabled = true;
    }

    @Override
    public int getPortID() {
        return portID;
    }

    @Override
    public void setPower(double power, RobotModuleBase operatorModule) {
        if (!isOwner(operatorModule))
            return;
        talonFXInstance.set(power * this.powerAndEncoderScaleFactor);
        currentPower = power;
        enabled = true;
    }

    @Override
    public double getCurrentPower() {
        return currentPower;
    }

    @Deprecated
    public void setTargetedPosition(double targetedPosition, RobotModuleBase operatorModule) {
        if (!isOwner(operatorModule))
            return;
        talonFXInstance.setPosition(targetedPosition);
        talonFXInstance.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void onDestroy() {
        disableMotor(ownerModule); // disable the motor, operate as its owner
    }

    @Override
    public void setMotorZeroPowerBehavior(ZeroPowerBehavior behavior, RobotModuleBase operatorModule) {
        if (!isOwner(operatorModule) || this.zeroPowerBehavior == behavior)
            return;
        switch (behavior) {
            case BRAKE: {
                talonFXInstance.setNeutralMode(NeutralModeValue.Brake);
                break;
            }
            case RELAX: {
                talonFXInstance.setNeutralMode(NeutralModeValue.Coast);
                break;
            }
            default: {
                throw new IllegalArgumentException("unknown zero power behavior: " + behavior);
            }
        }
        this.zeroPowerBehavior = behavior;
    }

    @Override
    public void lockMotor(RobotModuleBase operatorModule) {
        setMotorZeroPowerBehavior(ZeroPowerBehavior.BRAKE, operatorModule);
        setPower(0, operatorModule);
    }

    @Override
    public void disableMotor(RobotModuleBase operatorModule) {
        if (!enabled) return; // if already enabled, just skip
        // System.out.println("<-- TalonFX | motor id " + portID + " disabled -->");
        setMotorZeroPowerBehavior(ZeroPowerBehavior.RELAX, operatorModule);
        setPower(0, operatorModule);
        enabled = false;
    }

    public TalonFX getMotorInstance() {
        return talonFXInstance;
    }

    @Override
    public void setZeroPosition(double zeroPosition) {
        this.zeroPosition = zeroPosition;
    }

    /** gets the current position, not in radian */
    @Override
    public double getEncoderPosition() {
        return getRawEncoderReading() - zeroPosition;
    }

    /** gets the current velocity, not in radian, but in per second */
    @Override
    public double getEncoderVelocity() {
        return talonFXInstance.getRotorVelocity().getValueAsDouble() * 2048 * powerAndEncoderScaleFactor;
    }

    @Override
    public double getRawEncoderReading() {
        return talonFXInstance.getRotorPosition().getValueAsDouble() * powerAndEncoderScaleFactor * 2048;
    }

    public EncoderMotorMechanism toEncoderAndMotorMechanism() {
        return new EncoderMotorMechanism(this,this);
    }
}
