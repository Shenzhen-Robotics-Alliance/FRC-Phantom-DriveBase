package frc.robot.Utils.MechanismControllers;

public interface MechanismController {
    double getMotorPower(double mechanismVelocity, double mechanismPosition);
}
