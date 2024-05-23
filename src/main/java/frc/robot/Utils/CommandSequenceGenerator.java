package frc.robot.Utils;

import frc.robot.RobotCore;
import frc.robot.Utils.SequentialCommandSegment;

import java.util.List;

/**
 * write auto stages in subclasses that implements this abstract class
 * */
public interface CommandSequenceGenerator {
    List<SequentialCommandSegment> getCommandSegments(RobotCore robotCore);
}
