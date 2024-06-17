package frc.robot.Services;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Modules.MatchFieldSimulation;

public class ManualStageMatchSimulationService extends RobotServiceBase {
    private final MatchFieldSimulation matchFieldSimulation;

    private final SendableChooser<MatchFieldSimulation.OpponentRobot.Mode> opponentRobot0ModeChooser, opponentRobot1ModeChooser, opponentRobot2ModeChooser;
    public ManualStageMatchSimulationService(MatchFieldSimulation matchFieldSimulation) {
        super("Match-Sim-Service");
        this.matchFieldSimulation = matchFieldSimulation;

        this.opponentRobot0ModeChooser = new SendableChooser<>();
        this.opponentRobot1ModeChooser = new SendableChooser<>();
        this.opponentRobot2ModeChooser = new SendableChooser<>();
    }

    private void addOptions(SendableChooser<MatchFieldSimulation.OpponentRobot.Mode> chooser, int id) {
        chooser.setDefaultOption(MatchFieldSimulation.OpponentRobot.Mode.DEACTIVATED.name(), MatchFieldSimulation.OpponentRobot.Mode.DEACTIVATED);
        chooser.addOption(MatchFieldSimulation.OpponentRobot.Mode.MANUALLY_CONTROLLED.name(), MatchFieldSimulation.OpponentRobot.Mode.MANUALLY_CONTROLLED);
        chooser.addOption(MatchFieldSimulation.OpponentRobot.Mode.AUTO_CYCLING.name(), MatchFieldSimulation.OpponentRobot.Mode.AUTO_CYCLING);

        chooser.onChange(
                (MatchFieldSimulation.OpponentRobot.Mode mode) -> matchFieldSimulation.getOpponentRobot(id).setMode(mode)
        );
    }

    @Override
    public void init() {
        reset();
    }

    @Override
    public void reset() {
        addOptions(opponentRobot0ModeChooser, 0);
        addOptions(opponentRobot1ModeChooser, 1);
        addOptions(opponentRobot2ModeChooser, 2);

        SmartDashboard.putData("Opponent Robot 0 Mode", opponentRobot0ModeChooser);
        SmartDashboard.putData("Opponent Robot 1 Mode", opponentRobot1ModeChooser);
        SmartDashboard.putData("Opponent Robot 2 Mode", opponentRobot2ModeChooser);

        SmartDashboard.putData("Reset Field", new InstantCommand(matchFieldSimulation::resetField));
    }

    @Override
    public void periodic() {

    }
}
