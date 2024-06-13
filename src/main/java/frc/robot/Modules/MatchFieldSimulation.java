package frc.robot.Modules;

import frc.robot.Utils.PhysicsSimulation.AllRealFieldPhysicsSimulation;
import frc.robot.Utils.RobotConfigReader;

// TODO: write this class to simulate other robots on field
public class MatchFieldSimulation extends RobotModuleBase {
    public final AllRealFieldPhysicsSimulation.HolomonicRobotPhysicsSimulation robotPhysicsSimulation;
    public final AllRealFieldPhysicsSimulation simulation;
    public MatchFieldSimulation(RobotConfigReader robotConfig) {
        super("Match-Field-Sim", true);

        this.simulation = new AllRealFieldPhysicsSimulation();
        final AllRealFieldPhysicsSimulation.RobotProfile robotProfile = new AllRealFieldPhysicsSimulation.RobotProfile(robotConfig);
        this.robotPhysicsSimulation = this.simulation.addRobot(new AllRealFieldPhysicsSimulation.HolomonicRobotPhysicsSimulation(robotProfile));
    }

    @Override
    public void init() {

    }

    @Override
    protected void periodic(double dt) {

    }

    public void updatePhysics(double dt) {
        simulation.update(dt);
    }
    @Override
    public void onReset() {

    }

    public void addNoteToHumanPlayerStation() {

    }
}
