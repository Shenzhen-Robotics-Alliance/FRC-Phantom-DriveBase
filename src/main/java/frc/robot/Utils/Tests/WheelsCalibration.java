package frc.robot.Utils.Tests;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.hal.SimEnum;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drivers.Encoders.CanCoder;
import frc.robot.Drivers.Motors.TalonFXMotor;
import frc.robot.Utils.RobotConfigReader;

public class WheelsCalibration implements SimpleRobotTest { // calibrate wheel
    private final XboxController testController;
    private final RobotConfigReader config;

    private enum Wheel {
        frontLeft, frontRight, backLeft, backRight
    }

    private final SendableChooser<WheelsCalibration.Wheel> wheelsSendableChooser = new SendableChooser<>();

    public WheelsCalibration(RobotConfigReader robotConfig) {
        testController = new XboxController(1);
        this.config = robotConfig;
    }

    @Override
    public void testStart() {
        for (WheelsCalibration.Wheel wheel : WheelsCalibration.Wheel.values())
            wheelsSendableChooser.addOption(wheel.name(), wheel);
        wheelsSendableChooser.setDefaultOption(WheelsCalibration.Wheel.frontLeft.name(), WheelsCalibration.Wheel.frontLeft);
        SmartDashboard.putData("select wheel to calibrate", wheelsSendableChooser);
    }

    @Override
    public void testPeriodic() {
        // new TalonFX((int) config.getConfig("hardware", wheelsSendableChooser.getSelected().name() + "WheelDriveMotor"))
        TalonFXMotor testDriveMotor = new TalonFXMotor(
                // new TalonFX((int) config.getConfig("hardware", wheelsSendableChooser.getSelected().name() + "WheelDriveMotor"))
                new TalonFX((int) config.getConfig("hardware", wheelsSendableChooser.getSelected().name() + "WheelDriveMotor"), "ChassisCanivore")
        );
        testDriveMotor.gainOwnerShip(null);

        // new TalonFX((int) config.getConfig("hardware", wheelsSendableChooser.getSelected().name() + "WheelSteerMotor"))
        TalonFXMotor testSteerMotor = new TalonFXMotor(
                // new TalonFX((int) config.getConfig("hardware", wheelsSendableChooser.getSelected().name() + "WheelSteerMotor"))
                new TalonFX((int) config.getConfig("hardware", wheelsSendableChooser.getSelected().name() + "WheelSteerMotor"), "ChassisCanivore")
        );
        testSteerMotor.gainOwnerShip(null);

        // new CANCoder((int)config.getConfig("hardware", wheelsSendableChooser.getSelected().name() + "WheelEncoder"))
        CanCoder testCanCoder = new CanCoder(
                // new CANCoder((int)config.getConfig("hardware", wheelsSendableChooser.getSelected().name() + "WheelEncoder"))
                new CANcoder((int) config.getConfig("hardware", wheelsSendableChooser.getSelected().name() + "WheelEncoder"), "ChassisCanivore")
        );

        SmartDashboard.putNumber("raw steer encoder reading", testCanCoder.getRawEncoderReading());
        SmartDashboard.putNumber("raw steer encoder velocity", testCanCoder.getEncoderVelocity());

        if (testController.getAButton())
            testDriveMotor.setPower(0.3, null);
        else
            testDriveMotor.disableMotor(null);

        if (testController.getBButton())
            testSteerMotor.setPower(0.3, null);
        else
            testSteerMotor.disableMotor(null);
    }
}
