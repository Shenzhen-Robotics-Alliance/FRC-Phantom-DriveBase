package frc.robot.AutoStagePrograms;

import frc.robot.RobotCore;
import frc.robot.Utils.CommandSequenceGenerator;
import frc.robot.Utils.SequentialCommandSegment;

import java.util.ArrayList;
import java.util.List;

public class DoNothingAuto implements CommandSequenceGenerator {
    @Override
    public List<SequentialCommandSegment> getCommandSegments(RobotCore robotCore) {
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();
        return commandSegments;
    }
}
