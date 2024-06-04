package frc.robot.Utils.Tests;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Utils.EasyDataFlow;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.PhysicsSimulation.FieldMaps.CrescendoDefault;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.Force;
import org.dyn4j.dynamics.Torque;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.Mass;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.World;

public class AllRealPhysicsSimulationTest implements SimpleRobotTest {
    private final XboxController xboxController = new XboxController(1);
    private World<Body> world;
    private Body robot;
    @Override
    public void testStart() {
        world = new World<>();
        robot = new Body();

        world.setGravity(0, 0);
        robot.addFixture(Geometry.createSquare(0.8));
        robot.setMass(new Mass(new Vector2(), 90, 1));
        robot.setLinearDamping(8);
        robot.setAngularDamping(1);
        robot.translate(2, 1);
        world.addBody(robot);
        new CrescendoDefault().addObstaclesToField(world);
    }

    private double previousTime = Timer.getFPGATimestamp();
    @Override
    public void testPeriodic() {
        robot.applyForce(new Force(xboxController.getRightX() * 90*4.6*8, xboxController.getRightY() * -90*4.6*8));
        robot.applyTorque(new Torque(xboxController.getLeftX() * 3));
        world.step(1, Timer.getFPGATimestamp() - previousTime);

        if (Vector2D.fromVector2(robot.getLinearVelocity()).getMagnitude() < 0.05)
            robot.setLinearVelocity(0, 0);
        EasyDataFlow.putRobot(robot);
        EasyDataFlow.putNumber("test", "vel", Vector2D.fromVector2(robot.getLinearVelocity()).getMagnitude());

        previousTime = Timer.getFPGATimestamp();
    }
}
