package frc.robot.Drivers.Motors;

import frc.robot.Modules.RobotModuleBase;

public interface Motor {
     enum ZeroPowerBehavior {
        BRAKE,
        RELAX
    }
    /** get the port of the current motor */
    int getPortID();

    /**
     * sets the power of the current motor, given the module that operates the motor
     */
    void setPower(double power, RobotModuleBase operatorModule);

    /** get the desired power set last time */
    double getCurrentPower();

    /** gain ownership to this motor */
    void gainOwnerShip(RobotModuleBase ownerModule);

    /** destroy the motor so the resources are released */
    void onDestroy();

    void setMotorZeroPowerBehavior(ZeroPowerBehavior behavior, RobotModuleBase operatorModule);

    /** make the motor to and movable, can be overridden by set power */
    void disableMotor(RobotModuleBase operatorModule);

    /** lock the motors, so they can't move, can be overridden by set power */
    void lockMotor(RobotModuleBase operatorModule);
}