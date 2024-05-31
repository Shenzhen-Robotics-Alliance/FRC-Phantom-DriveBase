package frc.robot.Utils.Tests;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DashboardDataTest implements SimpleRobotTest {
    StructArrayPublisher<SwerveModuleState> swerveStatesPublisher;
    @Override
    public void testStart() {
        swerveStatesPublisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic("test swerve drive" + "/swerveStates", SwerveModuleState.struct).publish();

        swerveStatesPublisher.set(new SwerveModuleState[] {
                new SwerveModuleState(1, Rotation2d.fromRadians(0)),
                new SwerveModuleState(1, Rotation2d.fromRadians(0)),
                new SwerveModuleState(1, Rotation2d.fromRadians(0)),
                new SwerveModuleState(1, Rotation2d.fromRadians(0))
        });

        SmartDashboard.putData("test swerve drive", builder -> {
            builder.setSmartDashboardType("SwerveDrive");

            builder.addDoubleProperty("Front Left Angle", () -> 0, null);
            builder.addDoubleProperty("Front Left Velocity", () -> 1, null);

            builder.addDoubleProperty("Front Right Angle", () -> 0, null);
            builder.addDoubleProperty("Front Right Velocity", () -> 1, null);

            builder.addDoubleProperty("Back Left Angle", () -> 0, null);
            builder.addDoubleProperty("Back Left Velocity", () -> 1, null);

            builder.addDoubleProperty("Back Right Angle", () -> 0, null);
            builder.addDoubleProperty("Back Right Velocity", () -> 1, null);

            builder.addDoubleProperty("Robot Angle", () -> Math.toRadians(90), null);
        });


    }

    @Override
    public void testPeriodic() {

    }
}
