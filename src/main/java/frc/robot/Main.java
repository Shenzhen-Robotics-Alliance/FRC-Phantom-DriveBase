package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The entry of the robot's program
 * 
 * @author Sam
 * @version 0.1
 */
public class Main {
    public static final boolean isSim = false;
    public static void main(String[] args) {
        System.out.printf("@@@@@@  void main(String[] args)  %n");
        RobotBase.startRobot(RobotShell::new);
    }
}