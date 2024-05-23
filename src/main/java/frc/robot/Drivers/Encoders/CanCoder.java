package frc.robot.Drivers.Encoders;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.Drivers.RobotDriverBase;
import frc.robot.Utils.MathUtils.AngleUtils;

public class CanCoder extends RobotDriverBase implements Encoder {
    private final CANcoder encoderInstance;
    private final double encoderScaleFactor;
    private double encoderZeroPosition;

    public CanCoder(CANcoder encoderInstance) {
        this(encoderInstance, false);
    }

    public CanCoder(CANcoder encoderInstance, Boolean reversed) {
        this.encoderInstance = encoderInstance;
        this.encoderZeroPosition = 0;
        this.encoderScaleFactor = reversed ? -1 : 1;
    }

    @Override
    public void setZeroPosition(double zeroPosition) {
        this.encoderZeroPosition = zeroPosition;
    }

    @Override
    public int getPortID() {
        return encoderInstance.getDeviceID();
    }

    /**
     * get the reading of this encoder
     * returns the angle of the encoder, notice zero is to the direct right
     * positive is counter-clockwise, just as how we do it on the coordinate system
     */
    @Override
    public double getEncoderPosition() {
        final double differenceFromZeroPosition = AngleUtils.getActualDifference(encoderZeroPosition, getRawEncoderReading());
        return AngleUtils.simplifyAngle(differenceFromZeroPosition * encoderScaleFactor);
    }

    /** the raw sensor reading, converted to radian */
    @Override
    public double getRawEncoderReading() {
        return encoderInstance.getAbsolutePosition().getValueAsDouble() * Math.PI * 2;
    }

    @Override
    public double getEncoderVelocity() {
        return encoderInstance.getVelocity().getValueAsDouble() * encoderScaleFactor * Math.PI * 2;
    }
}
