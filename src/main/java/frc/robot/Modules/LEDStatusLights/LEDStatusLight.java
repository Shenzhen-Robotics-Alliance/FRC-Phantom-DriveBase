package frc.robot.Modules.LEDStatusLights;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Modules.RobotModuleBase;
import frc.robot.Utils.LEDAnimation;
import frc.robot.Utils.RobotModuleOperatorMarker;

public abstract class LEDStatusLight extends RobotModuleBase {
    protected final AddressableLEDBuffer buffer;
    private final Timer t = new Timer();

    LEDAnimation animation;
    double hz;
    protected LEDStatusLight(int length) {
        super("LED-Status-Light", true);
        this.buffer = new AddressableLEDBuffer(length);
    }

    @Override
    public void init() {}

    @Override
    protected void periodic(double dt) {
        animation.play(buffer, t.get());
    }

    @Override
    public void onReset() {
        t.start();
        t.reset();
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

    public abstract AddressableLED getLEDInstance();
}
