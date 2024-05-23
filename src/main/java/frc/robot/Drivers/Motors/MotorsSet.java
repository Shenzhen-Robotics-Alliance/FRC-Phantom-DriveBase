package frc.robot.Drivers.Motors;

import frc.robot.Modules.RobotModuleBase;

import java.util.Arrays;
import java.util.List;

public class MotorsSet implements Motor {
    private List<Motor> motors;
    private double power = 0;
    public MotorsSet(Motor[] motors) {
        this(Arrays.stream(motors).toList());
    }
    public MotorsSet(List<Motor> motors) {
        this.motors = motors;
    }


    @Override
    public int getPortID() {
        return -1;
    }

    @Override
    public void setPower(double power, RobotModuleBase operatorModule) {
        for (Motor motor:motors)
            motor.setPower(power, operatorModule);
        this.power = power;
    }

    @Override
    public double getCurrentPower() {
        return this.power;
    }

    @Override
    public void gainOwnerShip(RobotModuleBase ownerModule) {
        for (Motor motor:motors)
            motor.gainOwnerShip(ownerModule);
    }

    @Override
    public void onDestroy() {

    }

    @Override
    public void setMotorZeroPowerBehavior(ZeroPowerBehavior behavior, RobotModuleBase operatorModule) {
        for (Motor motor:motors)
            motor.setMotorZeroPowerBehavior(behavior, operatorModule);
    }

    @Override
    public void disableMotor(RobotModuleBase operatorModule) {
        for (Motor motor:motors)
            motor.disableMotor(operatorModule);
    }

    @Override
    public void lockMotor(RobotModuleBase operatorModule) {
        for (Motor motor:motors)
            motor.lockMotor(operatorModule);
    }
}
