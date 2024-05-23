package frc.robot.Services;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Modules.Chassis.SwerveBasedChassis;
import frc.robot.Utils.PilotController;
import frc.robot.Utils.RobotConfigReader;

public class PilotChassis extends RobotServiceBase {
    /** the module of the robot's chassis */
    protected final SwerveBasedChassis chassis;

    protected final RobotConfigReader robotConfig;

    /** the desired heading of the robot */
    protected double smartRotationControlDesiredHeading;

    private final SendableChooser<SwerveBasedChassis.OrientationMode> orientationModeChooser= new SendableChooser<>();
    private final SwerveBasedChassis.OrientationMode defaultOrientationMode = SwerveBasedChassis.OrientationMode.FIELD;

    private final SendableChooser<SwerveBasedChassis.WheelOutputMode> wheelOutputModeChooser = new SendableChooser<>();
    private final SwerveBasedChassis.WheelOutputMode defaultSpeedOutputMode = SwerveBasedChassis.WheelOutputMode.PERCENT_POWER;

    private enum ControllerType {
        RM_BOXER,
        RM_POCKET,
        JOYSTICK,
        XBOX
    }
    private SendableChooser<ControllerType> controllerTypeSendableChooser = new SendableChooser<>();
    private static final ControllerType defaultControllerType = ControllerType.RM_POCKET;

    /**
     * creates a pilot chassis
     * @param chassis
     * @param robotConfig
     * */
    public PilotChassis(SwerveBasedChassis chassis, RobotConfigReader robotConfig) {
        super("pilotChassisService");
        this.chassis = chassis;
        this.robotConfig = robotConfig;
    }

    @Override
    public void init() {
        reset();

    }

    @Override
    public void reset() {
        /* add mode choosers */
        for (SwerveBasedChassis.OrientationMode mode: SwerveBasedChassis.OrientationMode.values())
            this.orientationModeChooser.addOption(mode.name(), mode);
        this.orientationModeChooser.setDefaultOption(defaultOrientationMode.name(), defaultOrientationMode);
        SmartDashboard.putData("orientation mode", orientationModeChooser);

        for (SwerveBasedChassis.WheelOutputMode mode: SwerveBasedChassis.WheelOutputMode.values())
            this.wheelOutputModeChooser.addOption(mode.name(), mode);
        this.wheelOutputModeChooser.setDefaultOption(defaultSpeedOutputMode.name(), defaultSpeedOutputMode);
        SmartDashboard.putData("speed control", wheelOutputModeChooser);

        for (ControllerType controllerType: ControllerType.values())
            this.controllerTypeSendableChooser.addOption(controllerType.name(), controllerType);
        this.controllerTypeSendableChooser.setDefaultOption(defaultControllerType.name(), defaultControllerType);
        SmartDashboard.putData("pilot controller type", controllerTypeSendableChooser);

        this.chassis.gainOwnerShip(this);

        lastRotationalInputTimeMillis = System.currentTimeMillis();
    }

    private void addResetChassisCommandButtonToDashboard() {
        SmartDashboard.putData("Reset Chassis", new InstantCommand(() -> {
            chassis.reset();
            /* make rotation maintenance target zero */
            smartRotationControlDesiredHeading = 0;
            /* start rotation maintenance immediately */
            lastRotationalInputTimeMillis = System.currentTimeMillis() - (long)(robotConfig.getConfig("chassis", "timeLockRotationAfterRotationalInputStops") * 1000);
            addResetChassisCommandButtonToDashboard();
        }));
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

        /* set the control and orientation mode for wheel speed */
        chassis.setWheelOutputMode(wheelOutputModeChooser.getSelected(), this);
        chassis.setOrientationMode(orientationModeChooser.getSelected(), this);

        /* read and process pilot's translation input */
        final int translationAutoPilotButton = (int)robotConfig.getConfig(controllerName, "translationAutoPilotButton");
        final int smartRotationControlButton = (int) robotConfig.getConfig(controllerName, "rotationAutoPilotButton");
        SwerveBasedChassis.ChassisTaskTranslation chassisTranslationalTask = new SwerveBasedChassis.ChassisTaskTranslation(
                SwerveBasedChassis.ChassisTaskTranslation.TaskType.SET_VELOCITY,
                /* if autopilot is on, we scale the input down by a factor so that we can search for the target */
                pilotController.getTranslationalStickValue().multiplyBy(pilotController.keyOnHold(translationAutoPilotButton) ? robotConfig.getConfig("chassis", "lowSpeedModeTranslationalCommandScale"):1)
        );
        chassis.setLowSpeedModeEnabled(pilotController.keyOnHold(translationAutoPilotButton), this);


        /* read and process the pilot's rotation inputs */
        /* turn it into a task */
        SwerveBasedChassis.ChassisTaskRotation chassisRotationalTask = new SwerveBasedChassis.ChassisTaskRotation(
                SwerveBasedChassis.ChassisTaskRotation.TaskType.SET_VELOCITY,
                pilotController.getRotationalStickValue()
        );

        /* when there is rotational input or there has been rotational input in the previous 0.3 seconds, we record the current heading of the chassis */
        if (pilotController.getRotationalStickValue() != 0)
            lastRotationalInputTimeMillis = System.currentTimeMillis();
        if (System.currentTimeMillis() - lastRotationalInputTimeMillis < robotConfig.getConfig("chassis", "timeLockRotationAfterRotationalInputStops") * 1000)
            smartRotationControlDesiredHeading = chassis.getChassisHeading();
        else if (pilotController.keyOnHold(smartRotationControlButton))
            /* or, when there is no rotational input and that the smart rotation control is on, we stay at the previous rotation */
            chassisRotationalTask = new SwerveBasedChassis.ChassisTaskRotation(
                    SwerveBasedChassis.ChassisTaskRotation.TaskType.FACE_DIRECTION,
                    smartRotationControlDesiredHeading
            );

        /* calls to the chassis module and pass the desired motion */
        chassis.setTranslationalTask(chassisTranslationalTask, this);
        chassis.setRotationalTask(chassisRotationalTask, this);

        /* lock the chassis if needed */
        final int lockChassisButtonPort = (int) robotConfig.getConfig(controllerName, "lockChassisButtonPort");
        chassis.setChassisLocked(pilotController.keyOnHold(lockChassisButtonPort), this);

        SmartDashboard.putNumber("rotation maintenance heading", Math.toDegrees(smartRotationControlDesiredHeading));
    }

    @Override
    public void onDestroy() {

    }
}
