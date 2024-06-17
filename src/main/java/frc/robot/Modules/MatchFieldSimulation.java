package frc.robot.Modules;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Modules.Chassis.SwerveDriveChassisLogic;
import frc.robot.Modules.PositionReader.RobotFieldPositionEstimator;
import frc.robot.Utils.*;
import frc.robot.Utils.MathUtils.BezierCurve;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.MechanismControllers.EnhancedPIDController;
import frc.robot.Utils.PhysicsSimulation.AllRealFieldPhysicsSimulation;

import java.util.ArrayList;
import java.util.List;


public class MatchFieldSimulation extends RobotModuleBase {
    private final RobotConfigReader robotConfig;
    public final AllRealFieldPhysicsSimulation.HolomonicRobotPhysicsSimulation robotPhysicsSimulation;
    public final AllRealFieldPhysicsSimulation simulation;
    private final AllRealFieldPhysicsSimulation.RobotProfile robotProfile;
    private final OpponentRobot[] opponentRobots;
    public MatchFieldSimulation(RobotConfigReader robotConfig) {
        super("Match-Field-Sim", true);

        this.robotConfig = robotConfig;
        this.simulation = new AllRealFieldPhysicsSimulation();
        this.robotProfile = new AllRealFieldPhysicsSimulation.RobotProfile(robotConfig);
        this.robotPhysicsSimulation = this.simulation.setMainRobot(new AllRealFieldPhysicsSimulation.HolomonicRobotPhysicsSimulation(robotProfile));

        this.opponentRobots = new OpponentRobot[3];
        for (int i =0; i < 3; i++) {
            final OpponentRobot opponentRobot = new OpponentRobot(robotProfile, robotConfig, new PilotController(new XboxController(2), robotConfig, "control-XBOX"), i);
            this.opponentRobots[i] = opponentRobot;
            this.simulation.addRobot(opponentRobot);
        }
    }

    @Override
    public void init() {
        onReset();
    }

    @Override
    protected void periodic(double dt) {
        simulateHumanPlayer();

        for (OpponentRobot opponentRobot:opponentRobots)
            opponentRobot.updateOpponentRobotBehavior(dt);
    }

    public void updatePhysics(double dt) {
        simulation.update(dt);
    }

    private void simulateHumanPlayer() {
        final Vector2D humanPlayerStationPosition = RobotFieldPositionEstimator.toActualPositionOnField(new Vector2D(new double[] {16.54-0.95, 0.85}));
        if (!humanPlayerStationHasNote(humanPlayerStationPosition))
            simulation.addNoteToField(humanPlayerStationPosition);
    }

    private boolean humanPlayerStationHasNote(Vector2D humanPlayerStationPosition) {
        for (AllRealFieldPhysicsSimulation.NoteOnField noteOnField : simulation.getNotesOnField())
            if (Vector2D.displacementToTarget(humanPlayerStationPosition, noteOnField.getFieldPosition()).getMagnitude() < 0.5)
                return true;

        return false;
    }

    @Override
    public void onReset() {
        onEnable();
        resetField();
    }

    @Override
    protected void onEnable() {
        for (OpponentRobot opponentRobot:opponentRobots)
            opponentRobot.resetOpponentRobotBehavior();
    }

    public OpponentRobot getOpponentRobot(int index) {
        return opponentRobots[index];
    }

    public void clearNotes() {
        simulation.removeAllNotesOnField();
    }

    public void resetField() {
        clearNotes();
        simulation.addNoteToField(new Vector2D(new double[] {2.90, 4.1}));
        simulation.addNoteToField(new Vector2D(new double[] {2.90, 5.55}));
        simulation.addNoteToField(new Vector2D(new double[] {2.90, 7}));

        simulation.addNoteToField(new Vector2D(new double[] {8.27, 0.75}));
        simulation.addNoteToField(new Vector2D(new double[] {8.27, 2.43}));
        simulation.addNoteToField(new Vector2D(new double[] {8.27, 4.1}));
        simulation.addNoteToField(new Vector2D(new double[] {8.27, 5.78}));
        simulation.addNoteToField(new Vector2D(new double[] {8.27, 7.46}));

        simulation.addNoteToField(new Vector2D(new double[] {13.64, 4.1}));
        simulation.addNoteToField(new Vector2D(new double[] {13.64, 5.55}));
        simulation.addNoteToField(new Vector2D(new double[] {13.64, 7}));
    }



    public static class OpponentRobot extends AllRealFieldPhysicsSimulation.HolomonicRobotPhysicsSimulation {
        private static final Vector2D[] opponentRedRobotsStartingPositions = new Vector2D[] {
            new Vector2D(new double[] {15.2, 7}),
            new Vector2D(new double[] {15.2, 4}),
            new Vector2D(new double[] {15.2, 2.5})
        };
        public enum Mode {
            MANUALLY_CONTROLLED,
            AUTO_CYCLING,
            DEACTIVATED
        }

        private final RobotConfigReader robotConfig;
        private final ChassisPositionController positionController;
        private final EnhancedPIDController rotationController;
        private static final double cycleSpeed = 3.5, pidInAdvanceTime = 0.18;

        private Mode mode;
        private final PilotController pilotController;
        private final int opponentRobotID;


        public OpponentRobot(AllRealFieldPhysicsSimulation.RobotProfile robotProfile, RobotConfigReader robotConfig, PilotController pilotController, int opponentRobotID) {
            super(robotProfile);

            this.robotConfig = robotConfig;
            this.pilotController = pilotController;
            this.opponentRobotID = opponentRobotID;

            this.positionController = new ChassisPositionController(SwerveDriveChassisLogic.getChassisTranslationPIDConfigs(robotConfig));
            this.rotationController = new EnhancedPIDController(new EnhancedPIDController.StaticPIDProfile(
                    Math.PI * 2,
                    0.6,
                    0.05,
                    Math.toRadians(35),
                    Math.toRadians(1),
                    0.1,
                    0,
                    0
            ));
            resetOpponentRobotBehavior();
        }

        private void resetOpponentRobotBehavior() {
            resetOpponentRobotPosition();
            this.mode = Mode.DEACTIVATED;
        }

        private void resetOpponentRobotPosition() {
            super.setRobotPosition(RobotFieldPositionEstimator.toActualPositionOnField(opponentRedRobotsStartingPositions[opponentRobotID]));
            super.setRobotRotation(RobotFieldPositionEstimator.toActualRobotRotation(new Rotation2D(Math.toRadians(90))));
        }

        public void updateOpponentRobotBehavior(double dt) {
            switch (mode) {
                case AUTO_CYCLING -> updateAutoCycle(dt);
                case MANUALLY_CONTROLLED -> updateOpponentPilotActions();
                case DEACTIVATED -> super.setRobotPosition(new Vector2D(new double[] {-10, -10}));
            }
        }

        private void updateOpponentPilotActions() {
            if (pilotController == null)
                return;
            pilotController.update();
            super.simulateChassisRotationalBehavior(pilotController.getRotationalStickValue());
            super.simulateChassisTranslationalBehaviorFieldOriented(
                    pilotController.getTranslationalStickValue()
                            .multiplyBy(RobotFieldPositionEstimator.getPilotFacing2D()) // apply field-centric drive
                            .multiplyBy(new Rotation2D(Math.PI)) // notice that pilot facing is reversed for opponent
            );
        }

        double t, totalTimeNeeded;
        List<BezierCurve> fullCyclePath = new ArrayList<>();
        private void initializeAutoCycle() {
            List<BezierCurve> halfWay = SequentialCommandFactory.getBezierCurvesFromPathFile("cycle path " + opponentRobotID);
            fullCyclePath = new ArrayList<>();
            fullCyclePath.addAll(halfWay);
            fullCyclePath.addAll(SequentialCommandFactory.reversePath(halfWay));

            totalTimeNeeded = 0;
            for (BezierCurve bezierCurve: fullCyclePath)
                totalTimeNeeded += bezierCurve.length / cycleSpeed;

            t = totalTimeNeeded / 2; // start by half way
            super.setRobotPosition(getCurrentDesiredPositionAndVelocityFromCurves()[0]);
        }

        private void updateAutoCycle(double dt) {
            final double
                    rotationTargetWhenMoving = RobotFieldPositionEstimator.toActualRobotRotation(opponentRobotID == 0 ? new Rotation2D(0) : new Rotation2D(-Math.toRadians(90))).getRadian(),
                    rotationTargetAtHumanPlayerStation = RobotFieldPositionEstimator.toActualRobotRotation(new Rotation2D(-Math.toRadians(30))).getRadian();
            t += dt;
            if (t > totalTimeNeeded)
                t -= totalTimeNeeded;
            final Vector2D[] currentDesiredPositionAndVelocity = getCurrentDesiredPositionAndVelocityFromCurves();
            final double desiredFacing = t < 0.5 || (totalTimeNeeded - t < 0.5) ? rotationTargetAtHumanPlayerStation : rotationTargetWhenMoving;
            rotationController.startNewTask(new EnhancedPIDController.Task(EnhancedPIDController.Task.TaskType.GO_TO_POSITION, desiredFacing));
            positionController.setDesiredPosition(
                    currentDesiredPositionAndVelocity[0]
                            .addBy(currentDesiredPositionAndVelocity[1].multiplyBy(pidInAdvanceTime)) // in-advance control
            );
            EasyDataFlow.putCurvesOnField("match-simulation/ opponent robot " + opponentRobotID + " cycle path", fullCyclePath);
            EasyDataFlow.putPosition("match-simulation/ opponent robot " + opponentRobotID + " desired position", currentDesiredPositionAndVelocity[0], new Rotation2D(desiredFacing));
            super.simulateChassisRotationalBehavior(rotationController.getMotorPower(super.getFacing().getRadian(), super.getAngularVelocity(), dt));
            super.simulateChassisTranslationalBehaviorFieldOriented(positionController.getCorrectionPower(super.getFieldPosition(), super.getFieldVelocity()));
        }

        /**
         * Assumption: 0 < t < totalTimeNeeded
         * */
        private Vector2D[] getCurrentDesiredPositionAndVelocityFromCurves() {
            Vector2D[] currentDesiredPositionAndVelocity = new Vector2D[2];
            double timeCount = 0;
            for (BezierCurve currentCurve:fullCyclePath) {
                final double currentCurveETA = currentCurve.length / cycleSpeed;
                if (timeCount + currentCurveETA > t) {
                    final double actualT = (t - timeCount) / currentCurveETA;
                    currentDesiredPositionAndVelocity[0] = currentCurve.getPositionWithLERP(actualT);
                    currentDesiredPositionAndVelocity[1] = currentCurve.getVelocityWithLERP(actualT);
                    break;
                }
                timeCount += currentCurveETA;
            }
            return currentDesiredPositionAndVelocity;
        }

        public void setMode(Mode mode) {
            if (mode == Mode.AUTO_CYCLING && (this.mode != Mode.AUTO_CYCLING))
                initializeAutoCycle();
            if (mode == Mode.MANUALLY_CONTROLLED && (this.mode == Mode.DEACTIVATED))
                resetOpponentRobotPosition();
            this.mode = mode;
        }
    }
}
