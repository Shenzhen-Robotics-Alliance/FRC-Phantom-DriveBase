package frc.robot.Drivers.DistanceSensors;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RangingMode;
import au.grapplerobotics.LaserCan.TimingBudget;

public class LaserCanSensor implements DistanceSensor {
    private final LaserCan laserCan;

    public LaserCanSensor(int portID) {
        laserCan = new LaserCan(0);

        try {
            laserCan.setRangingMode(RangingMode.SHORT);
            laserCan.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
        } catch (ConfigurationFailedException e) {
            e.printStackTrace();
        }
    }


    @Override
    public double getDistanceCM(double defaultValue) {
        return laserCan.getMeasurement() == null ? defaultValue : (laserCan.getMeasurement().distance_mm * 10);
    }
}
