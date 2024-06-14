package frc.robot.Modules;

import frc.robot.Modules.PositionReader.RobotFieldPositionEstimator;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.PhysicsSimulation.AllRealFieldPhysicsSimulation;
import frc.robot.Utils.RobotConfigReader;

import java.util.ArrayList;
import java.util.List;

public class MatchFieldSimulation extends RobotModuleBase {
    public final AllRealFieldPhysicsSimulation.HolomonicRobotPhysicsSimulation robotPhysicsSimulation;
    public final AllRealFieldPhysicsSimulation simulation;
    private final List<AllRealFieldPhysicsSimulation.HolomonicRobotPhysicsSimulation> opponentRobots;
    public MatchFieldSimulation(RobotConfigReader robotConfig) {
        super("Match-Field-Sim", true);

        this.simulation = new AllRealFieldPhysicsSimulation();
        final AllRealFieldPhysicsSimulation.RobotProfile robotProfile = new AllRealFieldPhysicsSimulation.RobotProfile(robotConfig);
        this.robotPhysicsSimulation = this.simulation.addRobot(new AllRealFieldPhysicsSimulation.HolomonicRobotPhysicsSimulation(robotProfile));
        this.opponentRobots = new ArrayList<>();
    }

    @Override
    public void init() {

    }

    @Override
    protected void periodic(double dt) {
        simulateHumanPlayer();
    }

    public void updatePhysics(double dt) {
        simulation.update(dt);
    }

    public void simulateHumanPlayer() {
        final Vector2D humanPlayerStationPosition = RobotFieldPositionEstimator.toActualPositionOnField(new Vector2D(new double[] {16.54-0.95, 0.85}));
        if (!humanPlayerStationHasNote(humanPlayerStationPosition))
            simulation.addNoteToField(humanPlayerStationPosition);
    }

    public boolean humanPlayerStationHasNote(Vector2D humanPlayerStationPosition) {
        for (AllRealFieldPhysicsSimulation.NoteOnField noteOnField : simulation.getNotesOnField())
            if (Vector2D.displacementToTarget(humanPlayerStationPosition, noteOnField.getFieldPosition()).getMagnitude() < 0.5) {
                System.out.println("human player station note: " + noteOnField.getFieldPosition());
                return true;
            }
        return false;
    }

    @Override
    public void onReset() {

    }
}
