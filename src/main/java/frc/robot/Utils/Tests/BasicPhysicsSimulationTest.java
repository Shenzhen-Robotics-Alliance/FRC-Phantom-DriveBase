package frc.robot.Utils.Tests;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Utils.EasyDataFlow;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.PhysicsSimulation.AllRealFieldPhysicsSimulation;
import org.dyn4j.dynamics.Force;

public class BasicPhysicsSimulationTest implements SimpleRobotTest {
    private final XboxController xboxController = new XboxController(1);
    private AllRealFieldPhysicsSimulation simulation;
    private AllRealFieldPhysicsSimulation.HolomonicRobotPhysicsSimulation robotPhysicsSimulation;
    @Override
    public void testStart() {
        simulation = new AllRealFieldPhysicsSimulation();

        robotPhysicsSimulation = new AllRealFieldPhysicsSimulation.HolomonicRobotPhysicsSimulation(new AllRealFieldPhysicsSimulation.RobotProfile(
                4.6,
                12,
                10,
                Math.toRadians(300),
                Math.toRadians(600),
                0.3,
                75,
                0.8,
                0.8
        ));
        robotPhysicsSimulation.resetMotion();
        robotPhysicsSimulation.setRobotPosition(new Vector2D(new double[] {1,2}));
        robotPhysicsSimulation.setRobotRotation(new Rotation2D(0));
        simulation.addRobot(robotPhysicsSimulation);
    }

    @Override
    public void testPeriodic() {
        // TODO: find out why this is not working
        robotPhysicsSimulation.setAtRest(false);
        final Vector2D desiredMotion = new Vector2D(new double[] {xboxController.getRightX(), -xboxController.getRightY()});
        if (desiredMotion.getMagnitude() > 0.05)
            robotPhysicsSimulation.applyForce(new Force(Vector2D.toVector2(
                desiredMotion.multiplyBy(robotPhysicsSimulation.profile.propellingForce))));
        else {
            if (Vector2D.fromVector2(robotPhysicsSimulation.getLinearVelocity()).getMagnitude() > 0.05 * robotPhysicsSimulation.profile.robotMaxVelocity)
                robotPhysicsSimulation.applyForce(new Force(Vector2D.toVector2(
                        new Vector2D(Vector2D.fromVector2(robotPhysicsSimulation.getLinearVelocity()).getHeading(), -robotPhysicsSimulation.profile.frictionForce)
                )));
            else
                robotPhysicsSimulation.setLinearVelocity(0, 0);
        }

        robotPhysicsSimulation.applyTorque(-xboxController.getLeftX() * 10 * robotPhysicsSimulation.getMass().getInertia() * Math.toRadians(270));
        if (Math.abs(xboxController.getLeftX()) < 0.05 && Math.abs(robotPhysicsSimulation.getAngularVelocity()) < Math.toRadians(5))
            robotPhysicsSimulation.setAngularVelocity(0);

        simulation.update(0.01);

        EasyDataFlow.putRobot(robotPhysicsSimulation);
        EasyDataFlow.putNumber("test", "robot Velocity", Vector2D.fromVector2(robotPhysicsSimulation.getLinearVelocity()).getMagnitude());
        EasyDataFlow.putNumber("test", "angular velocity", Math.toDegrees(robotPhysicsSimulation.getAngularVelocity()));
    }
}
