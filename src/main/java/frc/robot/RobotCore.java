package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drivers.Encoders.CanCoder;
import frc.robot.Drivers.IMUs.NavX2IMU;
import frc.robot.Drivers.IMUs.PigeonsIMU;
import frc.robot.Drivers.IMUs.SimpleGyro;
import frc.robot.Drivers.Motors.TalonFXMotor;
import frc.robot.Drivers.Visions.PhantomClient;
import frc.robot.Modules.Chassis.*;
import frc.robot.Modules.LEDStatusLights.AddressableLEDStatusLight;
import frc.robot.Modules.LEDStatusLights.LEDStatusLight;
import frc.robot.Modules.LEDStatusLights.SimulatedLEDStatusLight;
import frc.robot.Modules.PositionReader.*;
import frc.robot.Modules.RobotModuleBase;
import frc.robot.Services.RobotServiceBase;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.PhysicsSimulation.AllRealFieldPhysicsSimulation;
import frc.robot.Utils.RobotConfigReader;

/**
 *
 * the core of the robot, including all the modules that powers the module
 * note that services are not included in this field
 * */
public class RobotCore {
        private boolean initialized = false;
        private static final long printTimeIfTimeMillisExceeds = 20;

        public RobotConfigReader robotConfig;
        public SimpleGyro gyro;
        public RobotFieldPositionEstimator positionEstimator;
        public SwerveDriveChassisLogic chassis;
        public LEDStatusLight statusLight;

        private final List<String> configsToTune = new ArrayList<>(1);
        private final List<RobotModuleBase> modules;
        private List<RobotServiceBase> services;
        protected boolean wasEnabled;
        public final PowerDistribution powerDistributionPanel = new PowerDistribution(1, PowerDistribution.ModuleType.kCTRE);

        /**
         * creates a robot core
         * creates the instances of all the modules, but do not call init functions yet
         * */
        public RobotCore(String configName, boolean isSimulation) {
                try {
                        robotConfig = new RobotConfigReader(configName);
                } catch (RuntimeException e) {
                        System.out.println("<-- error while reading config, trying simulation mode... -->");
                        robotConfig = new RobotConfigReader(configName);
                }

                modules = new ArrayList<>();
                services = new ArrayList<>();
                if (isSimulation)
                        createRobotSim();
                else
                        createRobotReal();
        }

        private AllRealFieldPhysicsSimulation physicsSimulation;
        private void createRobotSim() {
                System.out.println("<-- Robot Core | creating robot in simulation... -->");
                physicsSimulation = new AllRealFieldPhysicsSimulation();

                final PositionEstimatorSimulation positionEstimatorSimulation = new PositionEstimatorSimulation(robotConfig);
                this.positionEstimator = positionEstimatorSimulation;
                modules.add(positionEstimator);
                this.statusLight = new SimulatedLEDStatusLight(155);
                modules.add(statusLight);

                final SwerveWheelSimulation
                        frontLeftWheel = new SwerveWheelSimulation(1, robotConfig, new Vector2D(new double[] { -0.6, 0.6 })),
                        backLeftWheel = new SwerveWheelSimulation(1, robotConfig, new Vector2D(new double[] { -0.6, -0.6 })),
                        frontRightWheel = new SwerveWheelSimulation(1, robotConfig, new Vector2D(new double[] { 0.6, 0.6 })),
                        backRightWheel = new SwerveWheelSimulation(1, robotConfig, new Vector2D(new double[] { 0.6, -0.6 }));
                modules.add(frontLeftWheel);
                modules.add(backLeftWheel);
                modules.add(frontRightWheel);
                modules.add(backRightWheel);
                this.chassis = new SwerveDriveChassisSimulation(frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel, positionEstimatorSimulation, physicsSimulation, robotConfig);
                modules.add(chassis);

                physicsSimulation.addNoteToField(new Vector2D(new double[] {16.54/2, 8.21/2}));
        }

        private void createRobotReal() {
                System.out.println("<-- Robot Core | creating real robot... -->");

                final SwerveWheel
                        frontLeftWheel = createSwerveWheel("frontLeft", 1, new Vector2D(new double[] { -0.6, 0.6 })),
                        backLeftWheel = createSwerveWheel("backLeft", 2, new Vector2D(new double[] { -0.6, -0.6 })),
                        frontRightWheel = createSwerveWheel("frontRight", 3, new Vector2D(new double[] { 0.6, 0.6 })),
                        backRightWheel = createSwerveWheel("backRight", 4, new Vector2D(new double[] { 0.6, -0.6 }));
                modules.add(frontLeftWheel);
                modules.add(backLeftWheel);
                modules.add(frontRightWheel);
                modules.add(backRightWheel);

                // this.gyro = new SimpleGyro(0, false, new PigeonsIMU((int) robotConfig.getConfig("hardware/gyroPort")));
                this.gyro = new SimpleGyro(0, true, new NavX2IMU());

                this.positionEstimator = new VisionSupportedOdometer(new SwerveWheel[] {frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel}, gyro, new PhantomClient()); // TODO create phantom client
                modules.add(positionEstimator);

                this.chassis = new SwerveDriveChassis(frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel, positionEstimator, robotConfig);
                modules.add(chassis);

                this.statusLight = new AddressableLEDStatusLight(new AddressableLED(0), 155);
                modules.add(statusLight);
        }

        private SwerveWheel createSwerveWheel(String name, int id, Vector2D wheelInstallationPosition) {
                if (robotConfig.getConfig("hardware/chassisOnCanivore") != 0)
                        return createSwerveWheelOnCanivore(name, id, wheelInstallationPosition);
                return new SwerveWheel(
                        new TalonFXMotor(new TalonFX( (int) robotConfig.getConfig("hardware/"+name+"WheelDriveMotor"))),
                        new TalonFXMotor(new TalonFX( (int) robotConfig.getConfig("hardware/"+name+"WheelSteerMotor")), robotConfig.getConfig("hardware/"+name+"WheelSteerMotorReversed") == 1),
                        new TalonFXMotor(new TalonFX((int) robotConfig.getConfig("hardware/"+name+"WheelDriveMotor"))),
                        new CanCoder(new CANcoder((int) robotConfig.getConfig("hardware/"+name+"WheelEncoder")),
                                robotConfig.getConfig("hardware/"+name+"WheelSteerEncoderReversed") == 1),
                        wheelInstallationPosition,
                        robotConfig, 
                        id, 
                        robotConfig.getConfig("hardware/"+name+"WheelZeroPosition") 
                                + (robotConfig.getConfig("hardware/"+name+"WheelSteerEncoderReversed") == 1 ? 
                                        (Math.PI / 2) : (-Math.PI / 2))
                );
        }

        private SwerveWheel createSwerveWheelOnCanivore(String name, int id, Vector2D wheelInstallationPosition) {
                return new SwerveWheel(
                        new TalonFXMotor(new TalonFX( (int) robotConfig.getConfig("hardware/"+name+"WheelDriveMotor"), "ChassisCanivore")),
                        new TalonFXMotor(new TalonFX( (int) robotConfig.getConfig("hardware/"+name+"WheelSteerMotor"), "ChassisCanivore"), robotConfig.getConfig("hardware/"+name+"WheelSteerMotorReversed") == 1),
                        new TalonFXMotor(new TalonFX((int) robotConfig.getConfig("hardware/"+name+"WheelDriveMotor"), "ChassisCanivore")),
                        new CanCoder(new CANcoder((int) robotConfig.getConfig("hardware/"+name+"WheelEncoder"), "ChassisCanivore"),
                                robotConfig.getConfig("hardware/"+name+"WheelSteerEncoderReversed") == 1),
                        wheelInstallationPosition,
                        robotConfig,
                        id,
                        robotConfig.getConfig("hardware/"+name+"WheelZeroPosition")
                                + (robotConfig.getConfig("hardware/"+name+"WheelSteerEncoderReversed") == 1 ?
                                (Math.PI / 2) : (-Math.PI / 2))
                );
        }

        /**
         * initializes the robot
         * note that this will take a little bit of time as it involves creating threads
         * it should be called once each competition, when the driver station connects to the robot
         * */
        public void initializeRobot() {
                System.out.println("<-- Robot | initializing robot... -->");
                /* initialize the modules and services */
                for (RobotModuleBase module:modules) {
                        module.init();
                        module.reset();
                        module.disable();
                }

                /* start the config tuning */
                addConfigsToTune();
                for (String config:configsToTune)
                        robotConfig.startTuningConfig(config);

                System.out.println("<-- Robot | robot initialized -->");
                this.initialized = true;
        }

        private void addConfigsToTune() {
                /* feed forward controller */
//                configsToTune.add("chassis/driveWheelFeedForwardRate");
//                configsToTune.add("chassis/driveWheelFrictionDefaultValue");
//                configsToTune.add("chassis/timeNeededToFullyAccelerate");

                /* steer PID */
//                configsToTune.add("chassis/steerWheelErrorTolerance");
//                configsToTune.add("chassis/steerWheelErrorStartDecelerate");
//                configsToTune.add("chassis/steerWheelMaximumPower");
//                configsToTune.add("chassis/steerWheelMinimumPower");
//                configsToTune.add("chassis/steerWheelFeedForwardTime");
//                configsToTune.add("chassis/steerCorrectionPowerRateAtZeroWheelSpeed");
//                configsToTune.add("chassis/steerCorrectionPowerFullWheelSpeed");
        }

        /**
         * resets the robot the current stage
         * this is called once at the start of each stage (auto or teleop)
         * @param services the robot services that will be used this stage
         * */
        public void startStage(List<RobotServiceBase> services) {
                if (!initialized)
                        initializeRobot();
                this.services = services;
                System.out.println("<-- Robot Core | starting current stage... -->");
                /* initialize the services */
                for (RobotServiceBase service:services)
                        service.init();
                /* reset the services */
                for (RobotServiceBase service: services)
                        service.reset();
                /* resume the modules that was paused */
                for (RobotModuleBase module: modules)
                        module.enable();

                wasEnabled = true;
                System.out.println("<-- Robot Core | current stage started -->");
        }

        /**
         * end the current stage
         * */
        public void stopStage() {
                System.out.println("<-- Robot | pausing robot... -->");
                this.wasEnabled = false;
                for (RobotModuleBase module: modules)
                        module.disable();
                this.services = new ArrayList<>();

                System.out.println("<-- Robot | robot paused... -->");
        }

        /**
         * called when the robot is enabled
         * */
        private long t = System.currentTimeMillis();
        public void updateRobot() {
                updateServices();
                updateModules();


                robotConfig.updateTuningConfigsFromDashboard();

                /* monitor the program's performance */
                SmartDashboard.putNumber("robot main thread delay", System.currentTimeMillis()-t);
                t = System.currentTimeMillis();

                testFunctions();
        }

        public void updateServices() {
                for (RobotServiceBase service : services) {
                        long dt = System.currentTimeMillis();
                        service.periodic();
                        if (System.currentTimeMillis() - dt > printTimeIfTimeMillisExceeds)
                                System.out.println("update service " + service.serviceName + " took longer than expected, time: " + (System.currentTimeMillis() - dt));
                }
        }

        public void updateModules() {
                for (RobotModuleBase module:modules) {
                        long dt = System.currentTimeMillis();
                        module.periodic();
                        if (System.currentTimeMillis() - dt > printTimeIfTimeMillisExceeds)
                                System.out.println("update module " + module.moduleName + " took longer than expected, time: " + (System.currentTimeMillis() - dt));
                }
        }

        private final XboxController xboxController = new XboxController(1);
        private boolean previouslyPressed = false;
        public void testFunctions() {
                if (xboxController.getXButton() && (!previouslyPressed))
                        physicsSimulation.launchNote(positionEstimator.getRobotPosition2D());
                previouslyPressed = xboxController.getXButton();
        }
}