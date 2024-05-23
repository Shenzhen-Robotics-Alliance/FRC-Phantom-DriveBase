package frc.robot.Drivers;

import frc.robot.Modules.RobotModuleBase;

/**
 * the interface standard for hardware drivers of the robot
 * 
 * @author Sam
 * @version 0.1
 */
public abstract class RobotDriverBase {
    protected RobotModuleBase ownerModule = null;

    /**
     * sets a module to be the current owner of this hardware
     */
    public void gainOwnerShip(RobotModuleBase ownerModule) {
        this.ownerModule = ownerModule;
    }

    /**
     * gets the module that currently owns this hardware
     */
    public RobotModuleBase getCurrentOwnerModule() {
        return this.ownerModule;
    }

    public boolean isOwner(RobotModuleBase currentModule) {
        return currentModule == null || getCurrentOwnerModule() == currentModule;
    }
}
