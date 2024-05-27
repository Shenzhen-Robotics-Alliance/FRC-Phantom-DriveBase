package frc.robot.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Map;

/**
 * a simple tool that deals with logs and dashboards
 * based on <a href="https://github.com/Mechanical-Advantage/AdvantageKit">Advantage Kit</a>
 * */
public class EasyDataFlow {
    private static final Map<String, EasyDoubleDataEntry> dataEntries = new HashMap<>();
    private static final Map<String, EasyPosePublisher> positionEntries = new HashMap<>();
    private static final Map<String, EasyPoseArrayPublisher> positionArrayEntries = new HashMap<>();
    private static final DataLog log = DataLogManager.getLog();

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
            Pose2d pose2d = new Pose2d(position.getX(), position.getY(), Rotation2d.fromRadians(rotation.getRadian()));
            if (publisher != null) this.publisher.set(pose2d);
            Logger.recordOutput(name, Pose2d.struct, pose2d);
        }
    }

    public static void putPosition(String name, Vector2D position, Rotation2D rotation) {
        if (!positionEntries.containsKey(name))
            positionEntries.put(name, new EasyPosePublisher(name));
        positionEntries.get(name).setPosition(position, rotation);
    }

    private static final class EasyPoseArrayPublisher {
        private StructArrayPublisher<Pose2d> publisher = null;
        private EasyPoseArrayPublisher(String name) {
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
                pose2ds[i] = new Pose2d(positions[i].getX(), positions[i].getY(), Rotation2d.fromRadians(rotations[i].getRadian()));
            this.publisher.set(pose2ds);
        }
    }

    public static void putPositionArray(String name, Vector2D[] positions, Rotation2D[] rotations) {
        if (!positionArrayEntries.containsKey(name))
            positionArrayEntries.put(name, new EasyPoseArrayPublisher(name));
        positionArrayEntries.get(name).setPositions(positions, rotations);
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
