package frc.robot.Drivers.DistanceSensors;

public interface DistanceSensor {
    default double getDistanceCM() {
        return getDistanceCM(0);
    }

    /**
     * get the newest distance sensor update
     * @return the distance of the obstacle ahead, in CM, -1 for invalid
     * */
    double getDistanceCM(double defaultValue);

    default void setEnabled(boolean enabled) {}

    default boolean errorDetected() {return false;}
}
