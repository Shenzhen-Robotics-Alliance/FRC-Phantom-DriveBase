package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drivers.Encoders.CanCoder;
import frc.robot.Drivers.IMUs.PigeonsIMU;
import frc.robot.Drivers.IMUs.SimpleGyro;
import frc.robot.Drivers.Motors.TalonFXMotor;
import frc.robot.Modules.LEDStatusLights;
import frc.robot.Modules.PositionReader.SwerveWheelPositionEstimator;
import frc.robot.Modules.PositionReader.SwerveWheelPositionEstimatorCurveOptimized;
import frc.robot.Modules.RobotModuleBase;
import frc.robot.Modules.Chassis.SwerveBasedChassis;
import frc.robot.Modules.Chassis.SwerveWheel;
import frc.robot.Services.RobotServiceBase;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.RobotConfigReader;

/**
 *
 * the core of the robot, including all the modules that powers the module
 * note that services are not included in this field
 * */
public class RobotCore {
        private static final long printTimeIfTimeMillisExceeds = 20;

        public RobotConfigReader robotConfig;
        public final SwerveWheel frontLeftWheel, backLeftWheel, frontRightWheel, backRightWheel;
        public final SimpleGyro gyro;
        public final SwerveWheelPositionEstimator positionReader;
        public final SwerveBasedChassis chassisModule;
        public final LEDStatusLights statusLight;

        private final List<String> configsToTune = new ArrayList<>(1);
        private final List<RobotModuleBase> modules;
        private List<RobotServiceBase> services;
        protected boolean wasEnabled;

        /**
         * creates a robot core
         * creates the instances of all the modules, but do not call init functions yet
         * */
        public RobotCore(String configName) {
                System.out.println("<-- Robot Core | creating robot... -->");
                modules = new ArrayList<>();
                services = new ArrayList<>();

                try {
                        robotConfig = new RobotConfigReader(configName);
                } catch (RuntimeException e) {
                        System.out.println("<-- error while reading config, trying simulation mode... -->");
                        robotConfig = new RobotConfigReader(configName);
                }

                frontLeftWheel = createSwerveWheel("frontLeft", 1, new Vector2D(new double[] { -0.6, 0.6 }));
                modules.add(frontLeftWheel);

                backLeftWheel = createSwerveWheel("backLeft", 2, new Vector2D(new double[] { -0.6, -0.6 }));
                modules.add(backLeftWheel);

                frontRightWheel = createSwerveWheel("frontRight", 3, new Vector2D(new double[] { 0.6, 0.6 }));
                modules.add(frontRightWheel);

                backRightWheel = createSwerveWheel("backRight", 4, new Vector2D(new double[] { 0.6, -0.6 }));
                modules.add(backRightWheel);

                this.gyro = new SimpleGyro(0, false, new PigeonsIMU((int) robotConfig.getConfig("hardware/gyroPort")));

                final SwerveWheel[] swerveWheels = new SwerveWheel[] {frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel};
                positionReader = new SwerveWheelPositionEstimator(swerveWheels, gyro);
                modules.add(positionReader);

                SwerveWheelPositionEstimatorCurveOptimized testPositionEstimator = new SwerveWheelPositionEstimatorCurveOptimized(swerveWheels, gyro);
                modules.add(testPositionEstimator);

                this.chassisModule = new SwerveBasedChassis(swerveWheels, gyro, robotConfig, positionReader);
                modules.add(chassisModule);

                final Map<Integer, Vector2D> speakerTargetAprilTagReferences = new HashMap<>(), amplifierTargetAprilTagReferences = new HashMap<>(), noteTargetReferences = new HashMap<>();
                speakerTargetAprilTagReferences.put(4, new Vector2D(new double[] {0, -0.2}));
                // speakerTargetAprilTagReferences.put(3, new Vector2D(new double[] {-0.5,0}));
                // amplifierTargetAprilTagReferences.put(5, new Vector2D(new double[] {0, 0}));
                speakerTargetAprilTagReferences.put(7, new Vector2D(new double[] {0, -0.2}));
                // speakerTargetAprilTagReferences.put(8, new Vector2D(new double[] {-0.5,0}));
                // amplifierTargetAprilTagReferences.put(6, new Vector2D(new double[] {0, 0}));

                noteTargetReferences.put(1, new Vector2D()); // the id of note is always 0, and the note is itself the reference so the relative position is (0,0)

                this.statusLight = new LEDStatusLights(new AddressableLED(0), new AddressableLEDBuffer(155)); modules.add(statusLight);
                // this.statusLight = null;
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
                        module.disable();
                }

                /* start the config tuning */
                addConfigsToTune();
                for (String config:configsToTune)
                        robotConfig.startTuningConfig(config);

                System.out.println("<-- Robot | robot initialized -->");
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

                /* arm pid */
//                configsToTune.add("arm/maximumPower");
//                configsToTune.add("arm/errorStartDecelerate");
//                configsToTune.add("arm/errorTolerance");
//                configsToTune.add("arm/feedForwardTime");
//                configsToTune.add("arm/errorAccumulationProportion");
//                configsToTune.add("arm/maxAcceleration");
//                configsToTune.add("arm/maxVelocity");
//                configsToTune.add("arm/inAdvanceTime");
//                configsToTune.add("arm/errorToleranceAsInPosition");

                /* arm positions */
                configsToTune.add("arm/position-DEFAULT");
                configsToTune.add("arm/position-INTAKE");
                configsToTune.add("arm/position-SHOOT_NOTE");
                configsToTune.add("arm/position-SCORE_AMPLIFIER");

                configsToTune.add("shooter/defaultShootingRPM");
//                configsToTune.add("shooter/speedControllerProportionGain");
//                configsToTune.add("shooter/speedControllerFeedForwardGain");

                configsToTune.add("shooter/shooterRPM0");
                configsToTune.add("shooter/armAngle0");
                configsToTune.add("shooter/shooterRPM1");
                configsToTune.add("shooter/armAngle1");
                configsToTune.add("shooter/shooterRPM2");
                configsToTune.add("shooter/armAngle2");
                configsToTune.add("shooter/shooterRPM3");
                configsToTune.add("shooter/armAngle3");
                configsToTune.add("shooter/shooterRPM4");
                configsToTune.add("shooter/armAngle4");
                configsToTune.add("shooter/shooterRPM5");
                configsToTune.add("shooter/armAngle5");
        }

        /**
         * resets the robot the current stage
         * this is called once at the start of each stage (auto or teleop)
         * @param services the robot services that will be used this stage
         * */
        public void startStage(List<RobotServiceBase> services) {
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
}