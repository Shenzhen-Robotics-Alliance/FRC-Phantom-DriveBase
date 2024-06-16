package frc.robot.Services;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Modules.MatchFieldSimulation;

// TODO: here add score board and human player logic
public class MatchSimulationService extends RobotServiceBase {
    private final MatchFieldSimulation matchFieldSimulation;
    private final XboxController coachController;
    public MatchSimulationService(MatchFieldSimulation matchFieldSimulation, XboxController coachController) {
        super("Match-Sim-Service");
        this.matchFieldSimulation = matchFieldSimulation;
        this.coachController = coachController;
    }

    @Override
    public void init() {
        reset();
    }

    @Override
    public void reset() {

    }

    @Override
    public void periodic() {

    }
}
