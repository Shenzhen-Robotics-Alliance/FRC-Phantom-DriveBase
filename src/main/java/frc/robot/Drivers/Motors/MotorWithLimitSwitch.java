package frc.robot.Drivers.Motors;

import frc.robot.Modules.RobotModuleBase;

public class MotorWithLimitSwitch implements Motor {
    private LimitSwitch positiveDirectionLimitSwitch = null, negativeDirectionLimitSwitch = null;
    private final Motor originalMotor;
    private final Thread updateThread;
    private boolean destroyed;
    public MotorWithLimitSwitch(Motor originalMotor) {
        this.originalMotor = originalMotor;
        updateThread = new Thread(() -> {
            while (!destroyed) {
                if (positiveDirectionLimitSwitch != null && positiveDirectionLimitSwitch.limitReached() && originalMotor.getCurrentPower() > 0) {
                    originalMotor.disableMotor(null);
                    // System.out.println("positive limit reached");
                }
                if (negativeDirectionLimitSwitch != null && negativeDirectionLimitSwitch.limitReached() && originalMotor.getCurrentPower() < 0) {
                    originalMotor.disableMotor(null);
                    // System.out.println("negative limit reached");
                }
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        });
        destroyed = false;
        updateThread.start();
    }

    @Override
    public int getPortID() {
        return originalMotor.getPortID();
    }

    @Override
    public void setPower(double power, RobotModuleBase operatorModule) {
        if (positiveDirectionLimitSwitch != null && positiveDirectionLimitSwitch.limitReached() && power > 0) {
            // System.out.println("positive limit reached");
            return;
        }
        if (negativeDirectionLimitSwitch != null && negativeDirectionLimitSwitch.limitReached() && power < 0) {
            // System.out.println("negative limit reached");
            return;
        }
        originalMotor.setPower(power, operatorModule);
    }

    @Override
    public double getCurrentPower() {
        return originalMotor.getCurrentPower();
    }

    @Override
    public void gainOwnerShip(RobotModuleBase ownerModule) {
        originalMotor.gainOwnerShip(ownerModule);
    }

    @Override
    public void onDestroy() {
        originalMotor.onDestroy();
        this.destroyed = true;
        try {
            updateThread.join();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void setMotorZeroPowerBehavior(ZeroPowerBehavior behavior, RobotModuleBase operatorModule) {
        originalMotor.setMotorZeroPowerBehavior(behavior, operatorModule);
    }

    @Override
    public void disableMotor(RobotModuleBase operatorModule) {
        originalMotor.disableMotor(operatorModule);
    }

    @Override
    public void lockMotor(RobotModuleBase operatorModule) {
        originalMotor.lockMotor(operatorModule);
    }

    public void setPositiveDirectionLimitSwitch(LimitSwitch limitSwitch) {
        this.positiveDirectionLimitSwitch = limitSwitch;
    }

    public void setNegativeDirectionLimitSwitch(LimitSwitch limitSwitch) {
        this.negativeDirectionLimitSwitch = limitSwitch;
    }

    public interface LimitSwitch {
        boolean limitReached();
    }
}
