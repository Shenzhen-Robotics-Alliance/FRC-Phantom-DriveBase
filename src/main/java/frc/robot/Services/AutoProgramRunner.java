package frc.robot.Services;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Modules.Chassis.SwerveDriveChassisLogic;
import frc.robot.Modules.Chassis.SwerveDriveChassis;
import frc.robot.Utils.EasyDataFlow;
import frc.robot.Utils.MathUtils.*;
import frc.robot.Utils.RobotConfigReader;
import frc.robot.Utils.SequentialCommandSegment;

import java.util.ArrayList;
import java.util.List;

/**
 * runs a sequence of command segment
 * the auto stage of our robot is basically running the modules in this service, simulating a pilot's commands
 */
public class AutoProgramRunner extends RobotServiceBase {
    private List<SequentialCommandSegment> commandSegments;

    private final SwerveDriveChassisLogic chassis;
    private final BezierCurveScheduleGenerator scheduleGenerator;
    private final RobotConfigReader robotConfig;
    private int currentSegmentID;
    private SequentialCommandSegment.StaticSequentialCommandSegment currentCommandSegment;
    private BezierCurveSchedule currentPathSchedule;
    private final Timer dt = new Timer();
    private double currentSegmentRotationScheduleETA, rotationT, inAdvanceTime;

    public AutoProgramRunner(SwerveDriveChassisLogic chassis, RobotConfigReader robotConfig) {
        super("Auto-Program-Runner");
        this.chassis = chassis;
        this.robotConfig = robotConfig;
        this.scheduleGenerator = new BezierCurveScheduleGenerator(robotConfig);

        dt.start();
    }

    @Override
    public void init() {
        this.reset();
    }

    @Override
    public void updateConfigs() {
        inAdvanceTime = robotConfig.getConfig("auto", "inAdvanceTime");
    }

    private Rotation2D previousRotationTask = new Rotation2D(0);
    @Override
    public void periodic() {
        if (currentSegmentID == -1) {
            if (commandSegments.isEmpty())
                return;
            initiateSegment(0);
        }
        updateConfigs();
        chassis.setOrientationMode(SwerveDriveChassis.OrientationMode.FIELD, this);

        if (currentPathSchedule != null) {
            final double translationalT = currentPathSchedule.nextCheckPoint(dt.get() * currentCommandSegment.timeScale);
            final Vector2D inAdvanceSpaceWithoutConstrain =  currentPathSchedule.getVelocityWithLERP().multiplyBy(inAdvanceTime),
                    distanceLeft = Vector2D.displacementToTarget(currentPathSchedule.getPositionWithLERP(), currentPathSchedule.getPositionWithLERP(1)),
                    inAdvanceSpaceWithConstrain = new Vector2D(inAdvanceSpaceWithoutConstrain.getHeading(), Math.min(inAdvanceSpaceWithoutConstrain.getMagnitude(), distanceLeft.getMagnitude()));
            chassis.setTranslationalTask(new SwerveDriveChassis.ChassisTaskTranslation(
                            SwerveDriveChassis.ChassisTaskTranslation.TaskType.GO_TO_POSITION,
                            currentPathSchedule.getPositionWithLERP().addBy(
                                    currentSegmentID == commandSegments.size()-1 ?
                                            inAdvanceSpaceWithConstrain : inAdvanceSpaceWithoutConstrain)),
                    this);
            EasyDataFlow.putNumber("auto", "segment ID", currentSegmentID);
            EasyDataFlow.putNumber("auto", "translational scaled T", translationalT);
            EasyDataFlow.putNumber("auto", "position (x)", currentPathSchedule.getPositionWithLERP().getX());
            EasyDataFlow.putNumber("auto", "position (y)", currentPathSchedule.getPositionWithLERP().getY());
        }

        if (currentSegmentRotationScheduleETA != -1) {
            rotationT += dt.get() / currentSegmentRotationScheduleETA;
            double rotationTSyncedToTranslationT = rotationT;
            if (currentPathSchedule != null)
                rotationTSyncedToTranslationT = Math.min(currentPathSchedule.getT(), rotationTSyncedToTranslationT);
            chassis.setRotationalTask(new SwerveDriveChassis.ChassisTaskRotation(
                            SwerveDriveChassis.ChassisTaskRotation.TaskType.FACE_DIRECTION,
                            currentCommandSegment.getCurrentRotationWithLERP(rotationTSyncedToTranslationT)),
                    this);
            EasyDataFlow.putNumber("auto", "rotation T", rotationT);
            EasyDataFlow.putNumber("auto", "rotation (deg)", Math.toDegrees(currentCommandSegment.getCurrentRotationWithLERP(rotationTSyncedToTranslationT)));
            previousRotationTask = new Rotation2D(currentCommandSegment.getCurrentRotationWithLERP(rotationTSyncedToTranslationT));
        }

        if (currentPathSchedule != null) {
            EasyDataFlow.putPosition("/auto/currentAutoStagePosition", currentPathSchedule.getPositionWithLERP(), previousRotationTask);
        }


        currentCommandSegment.periodic.run();

        if (isCurrentSegmentComplete())
            nextSegment();

        dt.reset();
    }

    @Override
    public void onDestroy() {

    }

    @Override
    public void reset() {
        this.currentSegmentID = -1;
        chassis.gainOwnerShip(this);
        updateConfigs();
        commandSegments = new ArrayList<>();
    }

    private void nextSegment() {
        this.commandSegments.get(currentSegmentID).ending.run();

        if (currentSegmentID+1 < commandSegments.size())
            initiateSegment(currentSegmentID+1);
    }

    private void initiateSegment(int segmentID) {
        this.currentSegmentID = segmentID;
        currentCommandSegment = this.commandSegments.get(segmentID).embodyCurrentCommandSegment();

        if (!currentCommandSegment.initiateCondition.initiateOrSkip()) {
            System.out.println("skipping segment: " + segmentID);
            nextSegment();
        }

        currentCommandSegment.beginning.run();

        final boolean rotationSpecified = currentCommandSegment.startingRotation != null && currentCommandSegment.endingRotation != null;
        this.currentSegmentRotationScheduleETA = rotationSpecified ?
                scheduleGenerator.getTimeNeededToFinishRotationalSchedule(currentCommandSegment.startingRotation.getRadian(), currentCommandSegment.endingRotation.getRadian())
                : -1;
        rotationT = 0;

        if (currentCommandSegment.chassisMovementPath == null) return;
        this.currentPathSchedule = scheduleGenerator.generateTranslationalSchedule(currentCommandSegment.chassisMovementPath);
        chassis.gainOwnerShip(this);
        EasyDataFlow.putCurveOnField("auto/currentAutoPath", currentCommandSegment.chassisMovementPath);
    }

    public void scheduleCommandSegments(List<SequentialCommandSegment> commandSegments) {
        this.commandSegments = commandSegments;
    }

    public boolean isAutoStageComplete() {
        if (this.commandSegments.size() - this.currentSegmentID == 1
                && this.isCurrentSegmentComplete()) {
            EasyDataFlow.putCurveOnField("auto/currentAutoPath", new BezierCurve(new Vector2D(), new Vector2D()));
            EasyDataFlow.putPosition("/auto/currentAutoStagePosition", new Vector2D(new double[] {-100, -100}), new Rotation2D(0)); // remove the dashboard data
            return true;
        }
        return false;
    }

    public boolean isCurrentSegmentComplete() {
        SequentialCommandSegment currentSegment = this.commandSegments.get(this.currentSegmentID);
        final boolean translationalMovementFinished = currentPathSchedule == null || currentPathSchedule.isCurrentPathFinished();
        final boolean rotationalMovementFinished = currentSegmentRotationScheduleETA == -1 || rotationT >= 1;

//        if (!translationalMovementFinished)
//            System.out.println("<-- Auto Program Runner | waiting for path to finish -->");
//        else if (!rotationalMovementFinished)
//            System.out.println("<-- Auto Program Runner | waiting for rotation schedule to finish, t: " + rotationT + " -->");
//        else if (!currentSegment.isCompleteChecker.isComplete())
//            System.out.println("<-- Auto Program Runner | waiting for is complete checker to confirm complete -->");
        return translationalMovementFinished
                && rotationalMovementFinished
                && currentSegment.isCompleteChecker.isComplete();
    }
}
