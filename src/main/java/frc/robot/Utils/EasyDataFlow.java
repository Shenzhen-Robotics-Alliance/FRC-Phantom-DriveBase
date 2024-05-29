package frc.robot.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Map;
import java.util.Vector;

/**
 * a simple tool that deals with logs and dashboards
 * based on <a href="https://github.com/Mechanical-Advantage/AdvantageKit">Advantage Kit</a>
 * */
public class EasyDataFlow {
    private static final Map<String, EasyDoubleDataEntry> dataEntries = new HashMap<>();
    private static final Map<String, EasyPosePublisher> positionEntries = new HashMap<>();
    private static final Map<String, EasyPoseArrayPublisher> positionArrayEntries = new HashMap<>();
    private static final Map<String, EasySwerveStatesPositionPublisher> swerveStatesPublisher = new HashMap<>();
    private static final DataLog log = DataLogManager.getLog();
    private static final Field2d field = new Field2d();

    public static void matchStart() {
        SmartDashboard.putData("field", field);
    }

    public static void log(String message) {
        DataLogManager.log(message);
    }

    private static final class EasyPosePublisher {
        private StructPublisher<Pose2d> publisher = null;
        private final String name;
        private EasyPosePublisher(String name) {
            this.name = name;
            try {
                this.publisher = NetworkTableInstance.getDefault()
                        .getStructTopic(name, Pose2d.struct).publish();
            } catch (Exception ignored) {}
        }

        public void setPosition(Vector2D position, Rotation2D rotation) {
            Pose2d pose2d;
            if (position == null) pose2d = null;
            else
                pose2d = getPose(position, rotation);
            if (publisher != null) this.publisher.set(pose2d);
            Logger.recordOutput(name, Pose2d.struct, pose2d);
        }
    }

    public static void putRobot(Vector2D position, Rotation2D rotation) {
        putPosition("/chassis/robotPosition", position, rotation);
        field.setRobotPose(getPose(position, rotation));
    }

    public static void putPosition(String name, Vector2D position, Rotation2D rotation) {
        if (!positionEntries.containsKey(name))
            positionEntries.put(name, new EasyPosePublisher(name));
        positionEntries.get(name).setPosition(position, rotation);
    }

    private static final class EasyPoseArrayPublisher {
        private StructArrayPublisher<Pose2d> publisher = null;
        private final String name;
        private EasyPoseArrayPublisher(String name) {
            this.name = name;
            try {
                this.publisher = NetworkTableInstance.getDefault()
                        .getStructArrayTopic(name, Pose2d.struct).publish();
            } catch (Exception ignored) {}
        }

        public void setPositions(Vector2D[] positions, Rotation2D[] rotations) {
            if (positions.length != rotations.length) {
                log("Warning!!! set position in 2d array is provided with arrays with mismatched length");
                return;
            }
            final Pose2d[] pose2ds = new Pose2d[positions.length];
            for (int i = 0; i < pose2ds.length; i++)
                pose2ds[i] = getPose(positions[i], rotations[i]);
            this.publisher.set(pose2ds);
            Logger.recordOutput(name, Pose2d.struct, pose2ds);
        }
    }

    public static void putPositionArray(String name, Vector2D[] positions, Rotation2D[] rotations) {
        if (!positionArrayEntries.containsKey(name))
            positionArrayEntries.put(name, new EasyPoseArrayPublisher(name));
        positionArrayEntries.get(name).setPositions(positions, rotations);
    }

    private static Pose2d getPose(Vector2D position, Rotation2D rotation) {
        return new Pose2d(position.getX(), position.getY(), Rotation2d.fromRadians(rotation == null ? 0 : rotation.getRadian()));
    }

    private static final class EasySwerveStatesPositionPublisher {
        private SwerveModuleState[] swerveStates = new SwerveModuleState[4];
        private final String name;
        private StructArrayPublisher<SwerveModuleState> swerveStatesPublisher = null;
        private DoublePublisher robotFacingPublisher = null;
        public EasySwerveStatesPositionPublisher(String name) {
            this.name = name;
            try {
                swerveStatesPublisher = NetworkTableInstance.getDefault()
                        .getStructArrayTopic(name + "/swerveStates", SwerveModuleState.struct).publish();
                robotFacingPublisher = NetworkTableInstance.getDefault()
                        .getDoubleTopic(name + "/robotFacing").publish();
            } catch (Exception ignored) {}
        }

        public void setSwerveStates(SwerveModuleState[] swerveStates, Rotation2D robotFacing) {
            this.swerveStates = swerveStates;
            swerveStatesPublisher.set(swerveStates);
            robotFacingPublisher.set(robotFacing.getRadian());
            Logger.recordOutput(name + "/swerveStates", swerveStates);
            Logger.recordOutput(name + "/robotFacing", robotFacing.getRadian());
        }
    }


    public static void putSwerveState(String name, double frontLeftWheelSpeed, double frontLeftWheelFacing, double frontRightWheelSpeed, double frontRightWheelFacing,  double backLeftWheelSpeed, double backLeftWheelFacing,  double backRightWheelSpeed, double backRightWheelFacing,  Rotation2D robotFacing) {
        final SwerveModuleState[] swerveStates = new SwerveModuleState[4];
        swerveStates[0] = new SwerveModuleState(frontLeftWheelSpeed, Rotation2d.fromRadians(frontLeftWheelFacing - Math.toRadians(90)));
        swerveStates[1] = new SwerveModuleState(frontRightWheelSpeed, Rotation2d.fromRadians(frontRightWheelFacing - Math.toRadians(90)));
        swerveStates[2] = new SwerveModuleState(backLeftWheelSpeed, Rotation2d.fromRadians(backLeftWheelFacing - Math.toRadians(90)));
        swerveStates[3] = new SwerveModuleState(backRightWheelSpeed, Rotation2d.fromRadians(backRightWheelSpeed - Math.toRadians(90)));

        putSwerveState(name, swerveStates, robotFacing);
    }

    public static void putSwerveState(String name, Vector2D frontLeftWheelMotionToRobot, Vector2D frontRightWheelMotionToRobot, Vector2D backLeftWheelMotionToRobot, Vector2D backRightWheelMotionToRobot, Rotation2D robotFacing) {
        final SwerveModuleState[] swerveStates = new SwerveModuleState[4];
        swerveStates[0] = getSwerveState(frontLeftWheelMotionToRobot);
        swerveStates[1] = getSwerveState(frontRightWheelMotionToRobot);
        swerveStates[2] = getSwerveState(backLeftWheelMotionToRobot);
        swerveStates[3] = getSwerveState(backRightWheelMotionToRobot);

        putSwerveState(name, swerveStates, robotFacing);
    }

    private static SwerveModuleState getSwerveState(Vector2D wheelMotion) {
        if (wheelMotion.getMagnitude() == 0)
            return new SwerveModuleState(0, Rotation2d.fromRadians(0));
        return new SwerveModuleState(
                wheelMotion.getMagnitude(),
                Rotation2d.fromRadians(wheelMotion.getHeading() - Math.toRadians(90))
        );
    }

    public static void putSwerveState(String name, SwerveModuleState[] swerveModuleStates, Rotation2D robotFacing) {
        if (!swerveStatesPublisher.containsKey(name))
            swerveStatesPublisher.put(name, new EasySwerveStatesPositionPublisher(name));
        swerveStatesPublisher.get(name).setSwerveStates(swerveModuleStates, robotFacing);
    }

    private static final class EasyDoubleDataEntry {
        private DoubleLogEntry logEntry = null;
        private double value;
        public EasyDoubleDataEntry(String tab, String name, double startingValue) {
            this.value = startingValue;
            final String entryPath = "/" + tab + "/" + name;
            try {
                Shuffleboard.getTab(tab).addDouble(name, () -> this.value);
                this.logEntry = new DoubleLogEntry(log, entryPath);
            } catch (Exception ignored) {}
        }

        public void setValue(double value) {
            this.value = value;
            if (logEntry != null) this.logEntry.append(value);
        }
    }

    public static void putNumber(String tab, String title, double number) {
        final String path = tab + "/" + title;
        if (dataEntries.containsKey(path))
            dataEntries.get(path).setValue(number);
        else
            dataEntries.put(path, new EasyDoubleDataEntry(tab, title, number));
    }
}
