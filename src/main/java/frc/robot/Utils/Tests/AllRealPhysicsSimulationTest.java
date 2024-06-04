package frc.robot.Utils.Tests;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Utils.EasyDataFlow;
import frc.robot.Utils.MathUtils.AngleUtils;
import frc.robot.Utils.MathUtils.LookUpTable;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.PhysicsSimulation.FieldMaps.CrescendoDefault;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.dynamics.Force;
import org.dyn4j.dynamics.Torque;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.PhysicsWorld;
import org.dyn4j.world.World;

import java.awt.*;

public class AllRealPhysicsSimulationTest implements SimpleRobotTest {
    private final XboxController xboxController = new XboxController(1);
    private World<Body> world;
    private Body robot;
    @Override
    public void testStart() {
        world = new World<>();
        robot = new Body();

        final double robotMass = 75; // in kg
        world.setGravity(PhysicsWorld.ZERO_GRAVITY);
        BodyFixture bodyFixture = robot.addFixture(
                Geometry.createRectangle(0.8, 0.8),
                robotMass/(0.8*0.8),
                0.6,
                0.2);
        robot.setMass(MassType.NORMAL);
        robot.setLinearDamping(8);
        robot.setAngularDamping(8);
        robot.translate(2, 1);
        world.addBody(robot);
        new CrescendoDefault().addObstaclesToField(world);
    }

    private double previousTime = Timer.getFPGATimestamp();
    @Override
    public void testPeriodic() {
        final double
                robotMass = robot.getMass().getMass(),
                robotSlippingTime = 0.4,
                maxVelocity = 4.6,
                robtMaxLinearAcceleration = 8,
                maxCentripetalAcceleration = 12;
        final Vector2D currentVelocity = Vector2D.fromVector2(robot.getLinearVelocity()),
                desiredVelocity = new Vector2D(new double[] {xboxController.getRightX(), -xboxController.getRightY()}).multiplyBy(maxVelocity).multiplyBy(new Rotation2D(Math.toRadians(90)));

        final double
                speedDifference = desiredVelocity.getMagnitude() - currentVelocity.getMagnitude(),
                directionDifference = AngleUtils.getActualDifference(currentVelocity.getHeading(), desiredVelocity.getHeading()),
                maxFloorFriction = robotMass * (maxVelocity / robotSlippingTime),
                motorMaxPropellingForce = robtMaxLinearAcceleration * robotMass + maxFloorFriction;

        double motorCurrentPropellingForce = 0;
        if (speedDifference > 0.05)
            motorCurrentPropellingForce = LookUpTable.linearInterpretationWithBounding(0, motorMaxPropellingForce, desiredVelocity.getMagnitude(), maxFloorFriction, currentVelocity.getMagnitude());
        else if (speedDifference < -0.05)
            motorCurrentPropellingForce = 0;

        double steeringForceMag = 0;
        if (Math.abs(directionDifference) > Math.toDegrees(1))
            steeringForceMag = LookUpTable.linearInterpretationWithBounding(
                    0, 0,
                    Math.toRadians(120), maxCentripetalAcceleration * robotMass,
                    Math.abs(directionDifference)
            );

        final Vector2D motorPropellingForce = new Vector2D(currentVelocity.getHeading(), motorCurrentPropellingForce),
                frictionForce = new Vector2D(currentVelocity.getHeading(), -maxFloorFriction),
                steeringForce = new Vector2D(currentVelocity.getHeading() + Math.toRadians((directionDifference > 0 ? 90:-90)), steeringForceMag),
                rawNetForce = motorPropellingForce.addBy(frictionForce).addBy(steeringForce),
                netForce = new Vector2D(rawNetForce.getHeading(), Math.min(maxFloorFriction, rawNetForce.getMagnitude()));

        robot.applyForce(new Force(Vector2D.toVector2(netForce)));

//                timeNeededToAccelerateToTargetedSpeedWithCurrentPropellingForce = speedDifference / (motorCurrentPropellingForce / robotMass),
//                timeNeededToCompleteTurnToTargetedDirectionWithCurrentSteeringForce = steeringForce / (currentVelocity.getMagnitude() * robotMass);

//        robot.applyForce(new Force(xboxController.getRightX() * robot.getMass().getMass()*4.6*8, xboxController.getRightY() * -robot.getMass().getMass()*4.6*8));
        robot.applyTorque(new Torque(xboxController.getLeftX() * -Math.toRadians(300) * 8 * robot.getMass().getInertia()));
        world.step(1, Timer.getFPGATimestamp() - previousTime);

        if (Vector2D.fromVector2(robot.getLinearVelocity()).getMagnitude() < 0.05)
            robot.setLinearVelocity(0, 0);
        EasyDataFlow.putRobot(robot);
        EasyDataFlow.putNumber("test", "vel", Vector2D.fromVector2(robot.getLinearVelocity()).getMagnitude());
        EasyDataFlow.putNumber("test", "robot mass", robot.getMass().getMass());
        EasyDataFlow.putNumber("test", "robot inertia",robot.getMass().getInertia());


        previousTime = Timer.getFPGATimestamp();
    }
}
