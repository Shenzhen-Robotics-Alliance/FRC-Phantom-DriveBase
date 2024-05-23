package frc.robot.Utils;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.RobotShell;

import java.util.ConcurrentModificationException;
import java.util.HashMap;
import java.util.Map;

/**
 * an easier way to access custom tags of shuffleboard, but with the simple logic of SmartDashboard
 * by 5516 the "IRON MAPLE"
 * */
public class EasyShuffleBoard {
    private static final Map<String, Map<String, GenericEntry>> widgetsInTags = new HashMap<>();
    public static void putNumber(String tab, String title, double number) {
        if (RobotShell.isFormalCompetition)
            return;
        try {
            if (!widgetsInTags.containsKey(tab))
                widgetsInTags.put(tab, new HashMap<>());
            if (!widgetsInTags.get(tab).containsKey(title))
                addEntry(tab, Shuffleboard.getTab(tab), title, number, 0);
            if (widgetsInTags.get(tab).get(title) != null)
                widgetsInTags.get(tab).get(title).setDouble(number);
        } catch (Exception e) {
            e.printStackTrace();
            // System.out.print("<-- shuffleboard exception, ignoring... -->"); // TODO concurrent modification exception
        }
    }

    private static void addEntry(String tabName, ShuffleboardTab tab, String title, double number, int duplicate) {
        final String actualTitle =
                duplicate == 0 ? title : title + " (" + duplicate + ")";
        try {
            widgetsInTags.get(tabName).put(title, tab.add(actualTitle, number).getEntry());
        } catch (IllegalArgumentException ignored){
            addEntry(tabName, tab, title, number, duplicate+1);
        } catch (ConcurrentModificationException ignored) {}
    }

    public static double getNumber(String tag, String title, double defaultValue) {
        if (RobotShell.isFormalCompetition)
            return defaultValue;
        if (!widgetsInTags.containsKey(tag) || !widgetsInTags.get(tag).containsKey(title))
            return defaultValue; // in case the widget is nowhere to be found
        try {
            return widgetsInTags.get(tag).get(title).getDouble(defaultValue);
        } catch (Exception e) {
            // e.printStackTrace();
            return defaultValue;
        }

    }
}
