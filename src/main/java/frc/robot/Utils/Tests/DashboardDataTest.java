package frc.robot.Utils.Tests;

import frc.robot.Modules.LEDStatusLights.SimulatedLEDStatusLight;
import frc.robot.Utils.LEDAnimation;

public class DashboardDataTest implements SimpleRobotTest {
    final SimulatedLEDStatusLight simulatedLEDStatusLight = new SimulatedLEDStatusLight(100);
    @Override
    public void testStart() {
        simulatedLEDStatusLight.init();
        simulatedLEDStatusLight.reset();
        simulatedLEDStatusLight.enable();
    }

    @Override
    public void testPeriodic() {
        simulatedLEDStatusLight.setAnimation(new LEDAnimation.Slide(0,200, 255, 1, 0.8), null);
        simulatedLEDStatusLight.periodic();
    }
}
