package frc.robot.Services;

import frc.robot.Modules.RobotModuleBase;
import frc.robot.Utils.RobotModuleOperatorMarker;

import java.util.HashMap;

/**
 * The template for the classes that controls how the robot respond to the
 * pilot's commands
 * 
 * @author Sam
 * @version 0.1
 */
public abstract class RobotServiceBase extends RobotModuleOperatorMarker {
    /** the name of the service */
    public String serviceName;

    /**
     * initialization of robot service, just do super("your module name")
     */
    protected RobotServiceBase(String serviceName) {
        this.serviceName = serviceName;
    }

    /** called during initialization */
    abstract public void init();

    /** called to reset service to initial state, you can also call it by the end of init() */
    public abstract void reset();

    /** called during every loop */
    abstract public void periodic();

    /** update robot configs from robotConfigReader, used when debugging the robot override or nothing will be done */
    public void updateConfigs() {}

    /** called when the program ends */
    abstract public void onDestroy();
}
