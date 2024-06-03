package frc.robot.Utils.Tests;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Utils.EasyDataFlow;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.Force;
import org.dyn4j.dynamics.Torque;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.Mass;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.World;

public class PhysicsSimulationTest implements SimpleRobotTest {
    private final XboxController xboxController = new XboxController(1);
    private World<Body> world;
    private Body robot;
    @Override
    public void testStart() {
        world = new World<>();
        robot = new Body();

        world.setGravity(0, 0);
        robot.translate(1.0, 0.0);
        robot.addFixture(Geometry.createSquare(0.8));
        robot.setMass(new Mass(new Vector2(), 60, 1));
        world.addBody(robot);
        robot.setLinearDamping(10);
    }

    private double previousTime = Timer.getFPGATimestamp();
    @Override
    public void testPeriodic() {
        robot.applyForce(new Force(xboxController.getRightX() * 100, xboxController.getRightY() * 100));
        robot.applyTorque(new Torque(xboxController.getLeftX() * 3));
        world.step(1, Timer.getFPGATimestamp() - previousTime);

        EasyDataFlow.putRobot(robot);

        previousTime = Timer.getFPGATimestamp();
    }
}
