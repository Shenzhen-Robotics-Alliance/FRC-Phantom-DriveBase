package frc.robot.Drivers.Motors;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.Drivers.RobotDriverBase;
import frc.robot.Modules.RobotModuleBase;

public class VictorSPXMotor extends RobotDriverBase implements Motor {
    private final VictorSPX victorSPX;
    private double power = 0;
    private final boolean reversed;

    public VictorSPXMotor(VictorSPX victorSPX) {
        this(victorSPX, false);
    }
    public VictorSPXMotor(VictorSPX victorSPX, boolean reversed) {
        this.victorSPX = victorSPX;
        this.reversed = reversed;
    }

    @Override
    public int getPortID() {
        return victorSPX.getDeviceID();
    }

    @Override
    public void setPower(double power, RobotModuleBase operatorModule) {
        if (!isOwner(operatorModule))
            return;

        victorSPX.set(VictorSPXControlMode.PercentOutput, (this.power = power) * (reversed ? -1:1));
    }

    @Override
    public double getCurrentPower() {
        return power;
    }

    @Override
    public void onDestroy() {

    }

    @Override
    public void setMotorZeroPowerBehavior(ZeroPowerBehavior behavior, RobotModuleBase operatorModule) {

    }

    @Override
    public void disableMotor(RobotModuleBase operatorModule) {
        if (!isOwner(operatorModule))
            return;
        this.victorSPX.set(VictorSPXControlMode.PercentOutput, 0);
    }

    @Override
    public void lockMotor(RobotModuleBase operatorModule) {
        disableMotor(operatorModule);
    }
}
