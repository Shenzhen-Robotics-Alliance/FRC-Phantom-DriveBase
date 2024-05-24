package frc.robot.Modules.PositionReader;

import frc.robot.Modules.RobotModuleBase;
import frc.robot.Utils.MathUtils.Vector2D;


/**
 * TODO: the simulated position estimator for robot simulations
 * */
public class SimulationPositionsEstimator extends PositionEstimator {
    public SimulationPositionsEstimator() {
        super();
    }

    @Override
    public Vector2D getRobotVelocity2D() {
        return null;
    }

    @Override
    public double getRobotRotationalVelocity() {
        return 0;
    }

    @Override
    public Vector2D getRobotPosition2D() {
        return null;
    }

    @Override
    public Vector2D getRobotAcceleration2D() {
        return null;
    }

    @Override
    public double getRobotRotation() {
        return 0;
    }

    @Override
    public void resetRobotPosition() {

    }

    @Override
    public void resetRobotRotation() {

    }

    @Override
    public void setRobotPosition(Vector2D robotPosition) {

    }

    @Override
    public void setRobotRotation(double rotation) {

    }

    @Override
    public boolean isResultReliable() {
        return false;
    }

    @Override
    public void init() {

    }

    @Override
    protected void periodic(double dt) {

    }

    @Override
    public void onReset() {

    }

    @Override
    public void reset() {

    }
}
