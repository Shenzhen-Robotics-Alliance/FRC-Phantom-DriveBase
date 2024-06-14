package frc.robot.Modules;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Modules.PositionReader.RobotFieldPositionEstimator;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.PhysicsSimulation.AllRealFieldPhysicsSimulation;
import frc.robot.Utils.PilotController;
import frc.robot.Utils.RobotConfigReader;


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
            final OpponentRobot opponentRobot = new OpponentRobot(robotProfile, robotConfig, i);
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
            opponentRobot.update();
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
        for (OpponentRobot opponentRobot:opponentRobots)
            opponentRobot.resetPositionToStartingZone();

        this.opponentRobots[0].setCurrentTaskAsMovingManually(new XboxController(2)); // TODO: this is just for testing, call it in service
    }

    @Override
    protected void onEnable() {
        onReset();
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

        private Mode mode;
        private PilotController pilotController;
        private final int robotNum;
        public OpponentRobot(AllRealFieldPhysicsSimulation.RobotProfile robotProfile, RobotConfigReader robotConfig, int robotNum) {
            super(robotProfile);

            this.robotConfig = robotConfig;
            this.robotNum = robotNum;

            this.mode = Mode.DEACTIVATED;
            pilotController = null;
        }

        private void resetPositionToStartingZone() {
            super.setRobotPosition(RobotFieldPositionEstimator.toActualPositionOnField(opponentRedRobotsStartingPositions[robotNum]));
            super.setRobotRotation(RobotFieldPositionEstimator.toActualRobotRotation(new Rotation2D(Math.toRadians(90))));
        }

        public void update() {
            switch (mode) {
                case AUTO_CYCLING -> {
                    // TODO write this part
                }
                case MANUALLY_CONTROLLED -> {
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
                case DEACTIVATED -> super.setRobotPosition(new Vector2D(new double[] {-10, -10}));
            }
        }

        public void setCurrentTaskAsMovingManually(GenericHID driverController) {
            this.pilotController = new PilotController(driverController, robotConfig, "control-XBOX");
            this.mode = Mode.MANUALLY_CONTROLLED;
        }

        public void setCurrentTaskAsMovingThroughPaths() { // TODO write this
        }

        public void setCurrentTaskAsStayStill() {
            this.mode = Mode.DEACTIVATED;
        }
    }
}
