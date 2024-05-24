package frc.robot.Services;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Modules.Chassis.HolonomicChassis;
import frc.robot.Utils.PilotController;
import frc.robot.Utils.RobotConfigReader;

public class PilotChassis extends RobotServiceBase {
    /** the module of the robot's chassis */
    protected final HolonomicChassis chassis;

    protected final RobotConfigReader robotConfig;

    /** the desired heading of the robot */
    protected double smartRotationControlDesiredHeading;

    private final SendableChooser<HolonomicChassis.OrientationMode> orientationModeChooser= new SendableChooser<>();
    private final HolonomicChassis.OrientationMode defaultOrientationMode = HolonomicChassis.OrientationMode.FIELD;

    private enum ControllerType {
        RM_BOXER,
        RM_POCKET,
        JOYSTICK,
        XBOX,
        XBOX_LEFT_HANDED

    }
    private SendableChooser<ControllerType> controllerTypeSendableChooser = new SendableChooser<>();
    private static final ControllerType defaultControllerType = ControllerType.RM_POCKET;
    private final XboxController copilotGamePad;

    /**
     * creates a pilot chassis
     * @param chassis
     * @param robotConfig
     * */
    public PilotChassis(HolonomicChassis chassis, RobotConfigReader robotConfig, XboxController copilotGamePad) {
        super("pilotChassisService");
        this.chassis = chassis;
        this.robotConfig = robotConfig;
        this.copilotGamePad = copilotGamePad;
    }

    @Override
    public void init() {
        reset();
    }

    @Override
    public void reset() {
        /* add mode choosers */
        for (HolonomicChassis.OrientationMode mode: HolonomicChassis.OrientationMode.values())
            this.orientationModeChooser.addOption(mode.name(), mode);
        this.orientationModeChooser.setDefaultOption(defaultOrientationMode.name(), defaultOrientationMode);
        SmartDashboard.putData("orientation mode", orientationModeChooser);

        for (ControllerType controllerType: ControllerType.values())
            this.controllerTypeSendableChooser.addOption(controllerType.name(), controllerType);
        this.controllerTypeSendableChooser.setDefaultOption(defaultControllerType.name(), defaultControllerType);
        SmartDashboard.putData("pilot controller type", controllerTypeSendableChooser);

        this.chassis.gainOwnerShip(this);

        lastRotationalInputTimeMillis = System.currentTimeMillis();
    }

    private ControllerType previousSelectedController = null;
    protected PilotController pilotController;
    protected String controllerName;
    protected long lastRotationalInputTimeMillis;
    @Override
    public void periodic() {
        final ControllerType selectedController = controllerTypeSendableChooser.getSelected();

        controllerName = "control-" + selectedController;
        /* if the controller type is switched, we create a new instance */
        if (selectedController != previousSelectedController)
            pilotController = new PilotController(robotConfig, controllerName);
        previousSelectedController = selectedController;

        pilotController.update();

        /* set the orientation mode */
        chassis.setOrientationMode(orientationModeChooser.getSelected(), this);

        /* read and process pilot's translation input */
        final int translationAutoPilotButton = (int) robotConfig.getConfig(controllerName, "translationAutoPilotButton");
        final int smartRotationControlButton = (int) robotConfig.getConfig(controllerName, "rotationAutoPilotButton");
        HolonomicChassis.ChassisTaskTranslation chassisTranslationalTask = new HolonomicChassis.ChassisTaskTranslation(
                HolonomicChassis.ChassisTaskTranslation.TaskType.SET_VELOCITY,
                /* if autopilot is on, we scale the input down by a factor so that we can search for the target */
                pilotController.getTranslationalStickValue().multiplyBy(pilotController.keyOnHold(translationAutoPilotButton) ? robotConfig.getConfig("chassis", "lowSpeedModeTranslationalCommandScale"):1)
        );
        chassis.setLowSpeedModeEnabled(pilotController.keyOnHold(translationAutoPilotButton), this);


        /* read and process the pilot's rotation inputs */
        /* turn it into a task */
        HolonomicChassis.ChassisTaskRotation chassisRotationalTask = new HolonomicChassis.ChassisTaskRotation(
                HolonomicChassis.ChassisTaskRotation.TaskType.SET_VELOCITY,
                pilotController.getRotationalStickValue()
        );

        /* when there is rotational input or there has been rotational input in the previous 0.3 seconds, we record the current heading of the chassis */
        if (pilotController.getRotationalStickValue() != 0)
            lastRotationalInputTimeMillis = System.currentTimeMillis();
        if (System.currentTimeMillis() - lastRotationalInputTimeMillis < robotConfig.getConfig("chassis", "timeLockRotationAfterRotationalInputStops") * 1000)
            smartRotationControlDesiredHeading = chassis.getChassisHeading();
        else if (pilotController.keyOnHold(smartRotationControlButton))
            /* or, when there is no rotational input and that the smart rotation control is on, we stay at the previous rotation */
            chassisRotationalTask = new HolonomicChassis.ChassisTaskRotation(
                    HolonomicChassis.ChassisTaskRotation.TaskType.FACE_DIRECTION,
                    smartRotationControlDesiredHeading
            );

        /* calls to the chassis module and pass the desired motion */
        chassis.setTranslationalTask(chassisTranslationalTask, this);
        chassis.setRotationalTask(chassisRotationalTask, this);

        /* lock the chassis if needed */
        final int lockChassisButtonPort = (int) robotConfig.getConfig(controllerName, "lockChassisButtonPort");
        chassis.setChassisLocked(pilotController.keyOnHold(lockChassisButtonPort), this);

        if (copilotGamePad.getLeftStickButton() && copilotGamePad.getRightStickButton()) {
            copilotGamePad.setRumble(GenericHID.RumbleType.kBothRumble, 1);
            chassis.resetChassisPositionAndRotation();
        } else copilotGamePad.setRumble(GenericHID.RumbleType.kBothRumble, 0);

        SmartDashboard.putNumber("rotation maintenance heading", Math.toDegrees(smartRotationControlDesiredHeading));
    }

    @Override
    public void onDestroy() {

    }
}
