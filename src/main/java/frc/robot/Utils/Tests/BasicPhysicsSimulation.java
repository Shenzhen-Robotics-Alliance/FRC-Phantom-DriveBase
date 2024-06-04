package frc.robot.Utils.Tests;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Utils.EasyDataFlow;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.PhysicsSimulation.FieldMaps.CrescendoDefault;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.Force;
import org.dyn4j.dynamics.Torque;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.PhysicsWorld;
import org.dyn4j.world.World;

public class BasicPhysicsSimulation implements SimpleRobotTest {
    private final XboxController xboxController = new XboxController(1);
    private World<Body> world;
    private Body robot;
    private static final double
            maxAcceleration = 12,
            maxVelocity = 4.6,
            robotMass = 75,
            propellingForce = robotMass * maxAcceleration,
            frictionForce = 10 * robotMass,
            velocityDamping = maxAcceleration / maxVelocity;
    @Override
    public void testStart() {
        world = new World<>();
        robot = new Body();

        world.setGravity(PhysicsWorld.ZERO_GRAVITY);
        robot.addFixture(
                Geometry.createRectangle(0.8, 0.8),
                robotMass/(0.8*0.8),
                0.6,
                0.2
        );
        robot.setMass(MassType.NORMAL);
        robot.setLinearDamping(velocityDamping);
        robot.setAngularDamping(10);
        robot.translate(2, 1);
        world.addBody(robot);

        new CrescendoDefault().addObstaclesToField(world);
    }

    @Override
    public void testPeriodic() {
        robot.setAtRest(false);
        final Vector2D desiredMotion = new Vector2D(new double[] {xboxController.getRightX(), -xboxController.getRightY()});
        if (desiredMotion.getMagnitude() > 0.05)
            robot.applyForce(new Force(Vector2D.toVector2(
                desiredMotion.multiplyBy(propellingForce))));
        else {
            if (Vector2D.fromVector2(robot.getLinearVelocity()).getMagnitude() > 0.05 * maxVelocity)
                robot.applyForce(new Force(Vector2D.toVector2(
                        new Vector2D(Vector2D.fromVector2(robot.getLinearVelocity()).getHeading(), -frictionForce)
                )));
            else
                robot.setLinearVelocity(0, 0);
        }

        robot.applyTorque(-xboxController.getLeftX() * 10 * robot.getMass().getInertia() * Math.toRadians(270));
        if (Math.abs(robot.getAngularVelocity()) < Math.toRadians(2))
            robot.setAngularVelocity(0);
        world.step(1, 0.01);

        EasyDataFlow.putRobot(robot);
        EasyDataFlow.putNumber("test", "robot Velocity", Vector2D.fromVector2(robot.getLinearVelocity()).getMagnitude());
        EasyDataFlow.putNumber("test", "angular velocity", Math.toDegrees(robot.getAngularVelocity()));
    }
}
