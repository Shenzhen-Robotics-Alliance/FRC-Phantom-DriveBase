package frc.robot.Modules.LEDStatusLights;


import edu.wpi.first.wpilibj.AddressableLED;
import frc.robot.Modules.RobotModuleBase;

/**
 * TODO: move all the led status light references to here
 * */
public abstract class LEDStatusLight extends RobotModuleBase {
    protected LEDStatusLight() {
        super("LED-Status-Light", true);
    }

    public AddressableLED getLEDInstance() {
        return new AddressableLED(0);
    }
}
