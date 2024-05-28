package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AutoStagePrograms.DoNothingAuto;
import frc.robot.AutoStagePrograms.LeaveCommunity;
import frc.robot.Services.*;
import frc.robot.Utils.CommandSequenceGenerator;
import frc.robot.Utils.SequentialCommandSegment;
import frc.robot.Utils.Tests.*;
import org.littletonrobotics.junction.LoggedRobot;

import java.util.ArrayList;
import java.util.List;

public class RobotShell extends LoggedRobot {
    private static final int updateFreq = 50;

    private final XboxController copilotGamePad = new XboxController(1);

    private RobotCore robotCore;
    private AutoProgramRunner autoProgramRunner;
    private List<SequentialCommandSegment> commandSegments;
    private final SendableChooser<CommandSequenceGenerator> autoStageChooser = new SendableChooser<>();

    public RobotShell() {
        super(1.0/updateFreq);
    }


    /** called once when the robot powers on */
    @Override
    public void robotInit() {
        // System.out.println("<-- Robot Shell | robot init -->");
        robotCore = new RobotCore("fasterChassis", isSimulation());
    }

    /** called once when the driver station first connects to the robot */
    @Override
    public void driverStationConnected() {
        // System.out.println("<-- Robot Shell | driver station connected -->");
        robotCore.initializeRobot();

        autoProgramRunner = new AutoProgramRunner(robotCore.chassis, robotCore.robotConfig);

        addAutoStagePrograms();
        scheduleAutoCommands(autoStageChooser.getSelected());
        autoStageChooser.onChange(this::scheduleAutoCommands);
        SmartDashboard.putData("Select Auto", autoStageChooser);
    }

    private void scheduleAutoCommands(CommandSequenceGenerator commandSequenceGenerator) {
        System.out.println("<-- RobotShell || scheduling commands with auto program -->");
        if (commandSequenceGenerator!=null)
            this.commandSegments = commandSequenceGenerator.getCommandSegments(robotCore);
        System.out.println("<-- complete -->");
    }

    /** called repeatedly after the robot powers on, no matter enabled or not */
    @Override
    public void robotPeriodic() {
        // System.out.println("<-- Robot Shell | robot periodic -->");
    }

    /** called once when auto is selected and enable button is hit */
    @Override
    public void autonomousInit() {
        // System.out.println("<-- Robot Shell | autonomous init -->");
        startAutoStage(autoStageChooser.getSelected());
    }

    @Override
    public void autonomousPeriodic() {
        // System.out.println("<-- Robot Shell | auto periodic -->");
        robotCore.updateRobot();

//        if (autoProgramRunner.isAutoStageComplete())
//            robotCore.stopStage();
    }

    @Override
    public void teleopInit() {
        // System.out.println("<-- Robot Shell | teleop init -->");
        startManualStage();
    }

    @Override
    public void teleopPeriodic() {
        // System.out.println("<-- Robot Shell | teleop periodic -->");
        robotCore.updateRobot();
    }

    @Override
    public void disabledInit() {
        // System.out.println("<-- Robot Shell | disable init -->");
        stopStage();
        if (robotTest != null)
            robotTest.testEnd();
    }

    @Override
    public void disabledPeriodic() {
        // System.out.println("<-- Robot Shell | disabled periodic -->");
        robotCore.updateModules();
    }

    private SimpleRobotTest robotTest = null;
    @Override
    public void testInit() {
        // System.out.println("<-- Robot Shell | test init -->");
        if (robotTest == null)
            this.robotTest = new DataFlowTest(robotCore.robotConfig);
            // this.robotTest = new AddressableLEDTest(robotCore.statusLight.getLEDInstance());
        robotTest.testStart();
    }

    @Override
    public void testPeriodic() {
        // System.out.println("<-- Robot Shell | robot init -->");
        robotTest.testPeriodic();
    }

    private void startAutoStage(CommandSequenceGenerator autoStageProgram) {
        System.out.println("<-- Robot Shell | starting auto" + autoStageProgram + " -->");

        final List<RobotServiceBase> services = new ArrayList<>();
        services.add(autoProgramRunner);
        robotCore.startStage(services);
        autoProgramRunner.scheduleCommandSegments(commandSegments);
    }

    private void startManualStage() {
        final List<RobotServiceBase> services = new ArrayList<>();

        final PilotChassis pilotChassisService = new PilotChassis(robotCore.chassis, robotCore.robotConfig, copilotGamePad);
        services.add(pilotChassisService);

        robotCore.startStage(services);
    }

    private void stopStage() {
        robotCore.stopStage();
    }

    public void addAutoStagePrograms() {
        autoStageChooser.setDefaultOption("Leave Community", new LeaveCommunity());
        autoStageChooser.addOption("Do Nothing", new DoNothingAuto());
    }
}