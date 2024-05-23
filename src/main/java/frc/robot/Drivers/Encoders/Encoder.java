package frc.robot.Drivers.Encoders;

import frc.robot.Modules.RobotModuleBase;

public interface Encoder {
    /**
     * set a selected position to be the referred origin
     * such that, when the raw position is equal to this value, getEncoderPosition() returns 0.0
     * */
    void setZeroPosition(double zeroPosition);

    default void setCurrentPositionAsZeroPosition() {
        setZeroPosition(getRawEncoderReading());
    }

    /**
     * get the port of the current motor
     */
    int getPortID();

    /**
     * get the current position of the encoder
     * @return the amount of position, that the encoder has deviated from the referring zero position
     * */
    double getEncoderPosition();

    /** get the velocity of this encoder, in radian per second */
    double getEncoderVelocity();

    /**
     * get the raw encoder reading, despite the calibrated zero position
     * */
    double getRawEncoderReading();

    default boolean isEncoderAvailable() {return true;}
}
