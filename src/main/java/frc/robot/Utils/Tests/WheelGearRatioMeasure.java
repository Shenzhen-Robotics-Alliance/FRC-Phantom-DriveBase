package frc.robot.Utils.Tests;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Drivers.Motors.TalonFXMotor;

public class WheelGearRatioMeasure implements SimpleRobotTest {
    private final TalonFXMotor motor = new TalonFXMotor(new TalonFX(5));

    @Override
    public void testStart() {

    }

    @Override
    public void testPeriodic() {
        System.out.println(motor.getEncoderPosition());
    }
}
