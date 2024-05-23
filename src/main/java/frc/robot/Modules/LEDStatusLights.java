package frc.robot.Modules;

import edu.wpi.first.wpilibj.*;
import frc.robot.Modules.RobotModuleBase;
import frc.robot.Utils.LEDAnimation;
import frc.robot.Utils.RobotModuleOperatorMarker;

public class LEDStatusLights extends RobotModuleBase {
    final Timer t = new Timer();
    final AddressableLED led;
    final AddressableLEDBuffer buffer;

    LEDAnimation animation;
    double hz;
    public LEDStatusLights(AddressableLED led, AddressableLEDBuffer buffer) {
        super("LED-Status-Lights", true);
        this.led = led;
        this.buffer = buffer;

        if (led != null)
            led.setLength(buffer.getLength());
    }

    @Override
    public void init() {
        onReset();
    }

    @Override
    protected void periodic(double dt) {
        animation.play(buffer, t.get());
        if (led != null)
            led.setData(buffer);
    }

    @Override
    public void onReset() {
        t.start();
        t.reset();
        if (led!=null)
            led.start();
        animation = LEDAnimation.disabled;
    }

    @Override
    protected void onEnable() {
        animation = LEDAnimation.enabled;
        hz = 0.6;
    }

    @Override
    protected void onDisable() {
        onReset();
    }

    public void setAnimation(LEDAnimation animation, RobotModuleOperatorMarker operator) {
        if (!isOwner(operator)) return;
        this.animation = animation;
    }

    public void resetCurrentAnimation(RobotModuleOperatorMarker operatorMarker) {
        t.reset();
    }

    public AddressableLED getLed() {
        return this.led;
    }
}
