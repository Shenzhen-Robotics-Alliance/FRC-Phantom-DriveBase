package frc.robot.Utils.Tests;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Utils.LEDAnimation;

public class AddressableLEDTest implements SimpleRobotTest {
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(155);
    private final LEDAnimation animation =
            new LEDAnimation.Rainbow(2);
//            new LEDAnimation.Slide(0, 200, 255);
    private final Timer timer = new Timer();

    public AddressableLEDTest(AddressableLED led) {
        this.led = led;
    }

    @Override
    public void testStart() {
        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();
        timer.start();
    }

    @Override
    public void testPeriodic() {
        animation.play(buffer, (timer.get() * 0.5) % 1);
        led.setData(buffer);
    }

    @Override
    public void testEnd() {
        for (int i = 0; i < buffer.getLength(); i++)
            buffer.setRGB(i, 0, 0, 0);

        led.setData(buffer);
    }
}
