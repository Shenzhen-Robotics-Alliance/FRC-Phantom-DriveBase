package frc.robot.Modules.LEDStatusLights;

import edu.wpi.first.wpilibj.AddressableLED;

/**
 * TODO: led status light during simulation
 * */
public class SimulatedLEDStatusLight extends LEDStatusLight {
    public SimulatedLEDStatusLight(int length) {
        super(length);
    }

    @Override
    protected void periodic(double dt) {
        super.periodic(dt);
    }

    @Override
    public void onReset() {
        super.onReset();
    }

    @Override
    public AddressableLED getLEDInstance() {
        return null;
    }
}
