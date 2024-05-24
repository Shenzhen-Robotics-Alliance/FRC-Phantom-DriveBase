package frc.robot.AutoStagePrograms;

import frc.robot.RobotCore;
import frc.robot.Utils.CommandSequenceGenerator;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.SequentialCommandFactory;
import frc.robot.Utils.SequentialCommandSegment;

import java.util.ArrayList;
import java.util.List;

public class LeaveCommunity implements CommandSequenceGenerator {
    @Override
    public List<SequentialCommandSegment> getCommandSegments(RobotCore robotCore) {
        final List<SequentialCommandSegment> commandSegments = new ArrayList<>();
        final SequentialCommandFactory commandFactory = new SequentialCommandFactory(robotCore, "Leave Community", new Rotation2D(Math.toRadians(90)));
        commandSegments.add(commandFactory.calibratePositionEstimator());
        commandSegments.add(commandFactory.followSingleCurve("Leave Community", 0, new Rotation2D(Math.toRadians(90))));
        return commandSegments;
    }
}
