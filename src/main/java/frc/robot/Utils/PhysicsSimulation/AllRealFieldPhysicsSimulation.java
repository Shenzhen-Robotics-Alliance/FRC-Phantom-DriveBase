package frc.robot.Utils.PhysicsSimulation;

import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.PhysicsSimulation.FieldMaps.CrescendoDefault;
import frc.robot.Utils.RobotConfigReader;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.world.PhysicsWorld;
import org.dyn4j.world.World;

import java.util.ArrayList;
import java.util.List;

public class AllRealFieldPhysicsSimulation {
    private final World<Body> field;
    private final FieldCollisionMap map;
    private final List<HolomonicRobotPhysicsSimulation> robots;
    public AllRealFieldPhysicsSimulation() {
        this.robots = new ArrayList<>();
        field = new World<>();
        field.setGravity(PhysicsWorld.ZERO_GRAVITY);
        map = new CrescendoDefault();
        map.addObstaclesToField(field);
    }

    public HolomonicRobotPhysicsSimulation addRobot(HolomonicRobotPhysicsSimulation robot) {
        robots.add(robot);
        field.addBody(robot);
        return robot;
    }

    public void update(double dt) {
        this.field.step(1, dt);
    }

    public static final class RobotProfile {
        public final double
                robotMaxVelocity,
                robotMaxAcceleration,
                robotMass,
                propellingForce,
                frictionForce,
                linearVelocityDamping,
                maxAngularVelocity,
                maxAngularAcceleration,
                angularDamping,
                angularFrictionAcceleration,
                width,
                height;

        public RobotProfile(RobotConfigReader robotConfig) {
            this(
                    robotConfig.getConfig("chassis", "robotMaximumSpeed"),
                    robotConfig.getConfig("chassis", "motorMaximumAcceleration"),
                    robotConfig.getConfig("chassis", "floorFrictionAcceleration"),
                    Math.toRadians(robotConfig.getConfig("chassis", "robotMaxAngularVelocity")),
                    Math.toRadians(robotConfig.getConfig("chassis", "robotMaxAngularAcceleration")),
                    robotConfig.getConfig("chassis", "timeChassisStopsRotating"),
                    robotConfig.getConfig("chassis", "robotMass"),
                    robotConfig.getConfig("chassis", "width"),
                    robotConfig.getConfig("chassis", "height")
            );
        }
        public RobotProfile(double robotMaxVelocity, double robotMaxAcceleration, double floorFrictionAcceleration, double maxAngularVelocity, double maxAngularAcceleration, double timeChassisStopsRotating, double robotMass, double width, double height) {
            this.robotMaxVelocity = robotMaxVelocity;
            this.robotMaxAcceleration = robotMaxAcceleration;
            this.robotMass = robotMass;
            this.propellingForce = robotMaxAcceleration * robotMass;
            this.frictionForce = floorFrictionAcceleration * robotMass;
            this.linearVelocityDamping = robotMaxAcceleration / robotMaxVelocity;
            this.maxAngularVelocity = maxAngularVelocity;
            this.maxAngularAcceleration = maxAngularAcceleration;
            this.angularDamping = maxAngularAcceleration / maxAngularVelocity;
            this.angularFrictionAcceleration = maxAngularVelocity / timeChassisStopsRotating;
            this.width = width;
            this.height = height;
        }
    }

    public static class HolomonicRobotPhysicsSimulation extends Body {
        public final RobotProfile profile;
        public HolomonicRobotPhysicsSimulation(RobotProfile profile) {
            this.profile = profile;

            /* height and width is reversed */
            super.addFixture(
                    Geometry.createRectangle(profile.width, profile.height),
                    profile.robotMass / (profile.height * profile.width),
                    0.8,
                    0.08
            );

            super.setMass(MassType.NORMAL);
            super.setLinearDamping(profile.linearVelocityDamping);
            super.setAngularDamping(profile.angularDamping);
        }

        public void reset(Vector2D robotPositionOnField, Rotation2D robotFacing) {
            super.transform.setTranslation(robotPositionOnField.getX(), robotPositionOnField.getY());
            super.transform.setRotation(robotFacing.getRadian());
            setMotion(new Vector2D(), 0);
        }

        public void setMotion(Vector2D linearVelocity, double angularVelocity) {
            super.setLinearVelocity(Vector2D.toVector2(linearVelocity));
            super.setAngularVelocity(angularVelocity);
        }

        public Vector2D getFieldPosition() {
            return Vector2D.fromVector2(super.transform.getTranslation());
        }

        public Vector2D getFieldVelocity() {
            return Vector2D.fromVector2(super.linearVelocity);
        }

        public Rotation2D getFacing() {
            return Rotation2D.fromTransform(super.transform);
        }
    }

    public static abstract class FieldCollisionMap {
        private final List<Body> obstacles = new ArrayList<>();

        protected void addLinearObstacle(Vector2D startingPoint, Vector2D endingPoint) {
            addRectangularObstacle(
                    startingPoint.addBy(Vector2D.displacementToTarget(startingPoint, endingPoint).multiplyBy(0.5)),
                    Vector2D.displacementToTarget(startingPoint, endingPoint).getMagnitude(),
                    0.01,
                    new Rotation2D(Vector2D.displacementToTarget(startingPoint, endingPoint).getHeading())
            );
        }

        protected void addRectangularObstacle(Vector2D centerPosition, double width, double height, Rotation2D rotation) {
            final Body obstacle = new Body();
            obstacle.setMass(MassType.INFINITE);
            final BodyFixture fixture = obstacle.addFixture(Geometry.createRectangle(width, height));
            fixture.setFriction(0.8);
            fixture.setRestitution(0.4);
            obstacle.getTransform().setTranslation(Vector2D.toVector2(centerPosition));
            obstacle.getTransform().setRotation(rotation.getRadian());
            addCustomObstacle(obstacle);
        }

        protected void addCustomObstacle(Body customObstacle) {
            obstacles.add(customObstacle);
        }

        public void addObstaclesToField(World<Body> field) {
            for (Body obstacle:obstacles)
                field.addBody(obstacle);
        }
    }
}
