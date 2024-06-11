package frc.robot.Modules.LEDStatusLights;

import edu.wpi.first.wpilibj.*;
import frc.robot.Modules.RobotModuleBase;
import frc.robot.Utils.LEDAnimation;
import frc.robot.Utils.RobotModuleOperatorMarker;

public class AddressableLEDStatusLight extends LEDStatusLight {
    private final AddressableLED led;
    public AddressableLEDStatusLight(AddressableLED led, int length) {
        super(length);
        this.led = led;

        led.setLength(buffer.getLength());
    }

    @Override
    protected void periodic(double dt) {
        super.periodic(dt);
        led.setData(buffer);
    }

    @Override
    public void onReset() {
        super.onReset();
        led.start();
    }

    @Override
    public AddressableLED getLEDInstance() {
        return this.led;
    }
}
