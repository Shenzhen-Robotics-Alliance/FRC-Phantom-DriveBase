package frc.robot.Utils;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.HashMap;
import java.util.Map;

/**
 * a simple tool that deals with logs and dashboards
 * based on <a href="https://github.com/Mechanical-Advantage/AdvantageKit">Advantage Kit</a>
 * */
public class EasyDataFlow {
    private static final Map<String, EasyDoubleDataEntry> dataEntries = new HashMap<>();
    private static final DataLog log = DataLogManager.getLog();

    public static final class EasyDoubleDataEntry {
        private final SuppliedValueWidget<Double> networkTopic;
        private final DoubleLogEntry logEntry;
        private double value;
        public EasyDoubleDataEntry(String tap, String name, double startingValue) {
            this.value = startingValue;
            final String entryPath = "/" + tap + "/" + name;
            this.networkTopic = Shuffleboard.getTab(tap).addDouble(name, () -> this.value);
            this.logEntry = new DoubleLogEntry(log, entryPath);
        }

        public void setValue(double value) {
            this.value = value;
            this.logEntry.append(value);
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
