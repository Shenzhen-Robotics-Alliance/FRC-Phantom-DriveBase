package frc.robot.Drivers.Encoders;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Utils.MathUtils.AngleUtils;

/**
 * duty-cycle encoder on a DIO port
 * to calibrate:
 *  1.
 * */
public class DCAbsolutePositionEncoder implements Encoder {
    private final DutyCycleEncoder dutyCycleEncoder;
    private final double encoderFactor;
    private double zeroPosition;
    private double previousDistance;
    private final int channelID;
    public DCAbsolutePositionEncoder(int channelID) {
        this(channelID, false);
    }
    public DCAbsolutePositionEncoder(int channelID, boolean reversed) {
        this.dutyCycleEncoder = new DutyCycleEncoder(channelID);
        this.channelID = channelID;
        this.zeroPosition = 0;
        this.previousDistance = getRawEncoderReading();
        this.encoderFactor = reversed ? -1:1;
        dt.start();
    }

    /**
     * @param zeroPosition the absolute encoder value, or reading of the duty cycle multiply by Math.Pi * 2, when the encoder is at zero position
     * */
    @Override
    public void setZeroPosition(double zeroPosition) {
        this.zeroPosition = zeroPosition;
    }

    @Override
    public int getPortID() {
        return channelID;
    }

    /**
     * the rotter's current position, in radian
     * zero when the rotter is at zero position
     * */
    @Override
    public double getEncoderPosition() {
        return AngleUtils.simplifyAngle(
                AngleUtils.getActualDifference(zeroPosition, getRawEncoderReading())
                        * encoderFactor
        );
    }

    /**
     * the rotter's current velocity, in radian / second
     * positive is counter-clockwise
     * */
    private final Timer dt = new Timer();
    @Override
    public double getEncoderVelocity() {
        final double currentDistance = dutyCycleEncoder.getAbsolutePosition(),
                velocity = (currentDistance - previousDistance) / dt.get();
        dt.reset(); previousDistance = currentDistance;
        return velocity * encoderFactor;
    }

    /**
     * direction and zero position are not considered
     * */
    @Override
    public double getRawEncoderReading() {
        return dutyCycleEncoder.getAbsolutePosition() * Math.PI * 2;
    }

    @Override
    public boolean isEncoderAvailable() {
        dutyCycleEncoder.setConnectedFrequencyThreshold(900);
        return dutyCycleEncoder.isConnected();
    }

    public DutyCycleEncoder getRawSensorInstance() {
        return this.dutyCycleEncoder;
    }
}
