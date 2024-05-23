 package frc.robot.Drivers.DistanceSensors;

 import com.revrobotics.Rev2mDistanceSensor;
 import frc.robot.Main;

 public class Rev2mDistanceSensorEncapsulation implements DistanceSensor {
     private static final Rev2mDistanceSensor rev2mDistanceSensorInstance = Main.isSim ? null : new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kOnboard);
     public Rev2mDistanceSensorEncapsulation() {
         if (rev2mDistanceSensorInstance == null)
             return;
         rev2mDistanceSensorInstance.setMeasurementPeriod(0.02);
         setEnabled(true);
     }
     @Override
     public double getDistanceCM(double defualtValue) {
         // System.out.println("distance sensor update rate: " + 1/rev2mDistanceSensorInstance.getMeasurementPeriod());
         if (rev2mDistanceSensorInstance == null || !rev2mDistanceSensorInstance.isRangeValid()) {
             // System.out.println("range invalid, raw reading: " + rev2mDistanceSensorInstance.getRange(Rev2mDistanceSensor.Unit.kMillimeters));
             return defualtValue;
         }
         return rev2mDistanceSensorInstance.getRange(Rev2mDistanceSensor.Unit.kMillimeters) / 10.0;
     }

     @Override
     public void setEnabled(boolean enabled) {
         if (rev2mDistanceSensorInstance == null) return;
         final String start = enabled ? "starting" : "stopping", end = enabled ? "started" : "stopped";
         System.out.println("Rev2m Distance Sensor | " + start + " round robbin...");
         rev2mDistanceSensorInstance.setEnabled(enabled);
         System.out.println("Rev2m Distance Sensor | " + "round robbin " + end);

         System.out.println("Rev2m Distance Sensor | " + start + " automatic mode...");
         rev2mDistanceSensorInstance.setAutomaticMode(enabled);
         System.out.println("Rev2m Distance Sensor | " + "automatic mode " + end);
     }

     @Override
     public boolean errorDetected() {
         return !rev2mDistanceSensorInstance.isRangeValid();
     }
 }
