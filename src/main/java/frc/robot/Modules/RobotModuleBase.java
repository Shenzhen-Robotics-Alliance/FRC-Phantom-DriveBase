package frc.robot.Modules;

import frc.robot.Drivers.Motors.Motor;
import frc.robot.RobotShell;
import frc.robot.Utils.RobotModuleOperatorMarker;
import frc.robot.Utils.TimeUtils;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeoutException;

/**
 * The template for the classes that controls the different modules of the
 * robot, from chassis to arm and to aim
 * 
 * @author Sam
 * @version 0.1
 */
public abstract class RobotModuleBase extends RobotModuleOperatorMarker {
    private final ExecutorService periodicExecutor = Executors.newSingleThreadExecutor();
    /** the name of the module */
    public final String moduleName;

    private final boolean updateDuringDisabled;

    /** if the current module is enabled */
    private boolean enabled = true;

    /** the current owner services or modules of the module */
    protected RobotModuleOperatorMarker owner = null;

    /** always add motors to this list */
    protected List<Motor> motors = new ArrayList<>();

    private long previousTimeMillis;

    /**
     * public RobotModule(HashMap<String, RobotModule> dependenciesModules,
     * dependency object 1, dependency object 2, ...)
     */
    protected RobotModuleBase(String moduleName, boolean updateDuringDisabled) {
        this.moduleName = moduleName;
        this.updateDuringDisabled = updateDuringDisabled;
        previousTimeMillis = System.currentTimeMillis();
    }

    protected RobotModuleBase(String moduleName) {
        this(moduleName, false);
    }

    public abstract void init();

    /** called during every loop */
    protected abstract void periodic(double dt);

    public void periodic() {
        // System.out.println("<-- base periodic of " + moduleName + ", enabled: " + enabled + "-->");
        if (!enabled && !updateDuringDisabled)
            return;
        long newTimeMillis = System.currentTimeMillis();
        if (newTimeMillis == previousTimeMillis) {
            /* in case of dt=0 */
            TimeUtils.sleep(1);
            newTimeMillis = System.currentTimeMillis();
        }
        updateConfigs();
        // System.out.println("executing periodic");
        periodic((newTimeMillis - previousTimeMillis) / 1000.0f);
        this.previousTimeMillis = newTimeMillis;
        // System.out.println("<-- end of base periodic -->");
    }

    private RobotModuleBase getMarker() {
        return this;
    }

    /** update robot configs from robotConfigReader, used when debugging the robot override or nothing will be done */
    public void updateConfigs() {}

    /** called to reset module to initial state, you can also call it by the end of init() */
    public abstract void onReset();

    public void reset() {
        this.previousTimeMillis = System.currentTimeMillis();
        onReset();
        clearAccumulations();
    }

    /** called when the program ends */
    public void onDestroy() {}
    protected void onEnable() {}
    protected void onDisable() {}
    /** clear all the accumulating variables, such as integral in a PID */
    protected void clearAccumulations() {

    }

    public void enable() {
        if (enabled)
            return;
        System.out.println("<-- Module Base | enabling module " + moduleName + " -->");
        clearAccumulations();
        onEnable();
        this.enabled = true;
    }

    public void disable() {
        if (!enabled)
            return;
        System.out.println("<-- Module Base | disabling module " + moduleName + " -->");
        for (Motor motor:motors)
            motor.disableMotor(getMarker());
        onDisable();
        this.enabled = false;
    }

    /**
     * make a service or module the only owner of this module
     * @param owner the robot service or module that is desired to be the owner
     */
    public void gainOwnerShip(RobotModuleOperatorMarker owner) {
        this.owner = owner;
    }

    /**
     * check if a service or module has ownership to this module
     * @param operator the service or module that needs to be checked
     * @return whether it is the only owner of this module
     */
    public boolean isOwner(RobotModuleOperatorMarker operator) {
        return operator == null || operator == owner;
    }

    public long getPreviousUpdateTimeMillis() {
        return previousTimeMillis;
    }
}
