package frc.robot.Utils.PhysicsSimulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Utils.EasyDataFlow;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.PhysicsSimulation.FieldMaps.CrescendoDefault;
import frc.robot.Utils.RobotConfigReader;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.Mass;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.PhysicsWorld;
import org.dyn4j.world.World;

import java.util.ArrayList;
import java.util.List;

public class AllRealFieldPhysicsSimulation {
    private final World<Body> field;
    private final FieldCollisionMap map;
    private final List<HolomonicRobotPhysicsSimulation> robots;
    private final List<NoteOnField> notesOnField;
    public AllRealFieldPhysicsSimulation() {
        this.robots = new ArrayList<>();
        this.notesOnField = new ArrayList<>();
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

    public NoteOnField addNoteToField(Vector2D startingPosition) {
        final NoteOnField noteOnField = new NoteOnField(startingPosition);
        notesOnField.add(noteOnField);
        field.addBody(noteOnField);
        return noteOnField;
    }

    // TODO: the removal of a note (remove a note with a given constrain)

    public void update(double dt) {
        for (NoteOnField noteOnField:notesOnField)
            noteOnField.update();
        this.field.step(1, dt);

        EasyDataFlow.putPosition3dArray("notePositions", getNotesOnFieldPose3d());
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

    public static class NoteOnField extends Body {
        public static final double
                noteRadius = 0.1778,
                noteMass = 0.235,
                frictionAcceleration = -4,
                noteHeight = 0.05;
        public NoteOnField(Vector2D startingPosition) {
            /* height and width is reversed */
            BodyFixture bodyFixture = super.addFixture(Geometry.createCircle(noteRadius));
            bodyFixture.setFriction(0.8);
            bodyFixture.setRestitution(0.1);
            super.setMass(new Mass(new Vector2(), noteMass, Double.POSITIVE_INFINITY));
            super.translate(Vector2D.toVector2(startingPosition));
        }

        public void update() {
            super.setAtRest(false);
            if (getFieldVelocity().getMagnitude() > 0.3)
                super.applyForce(Vector2D.toVector2(new Vector2D(getFieldVelocity().getHeading(), frictionAcceleration * super.getMass().getMass())));
            else
                super.setLinearVelocity(0, 0);
        }

        public Vector2D getFieldPosition() {
            return Vector2D.fromVector2(super.transform.getTranslation());
        }

        public Vector2D getFieldVelocity() {
            return Vector2D.fromVector2(super.linearVelocity);
        }
    }

    public Pose3d[] getNotesOnFieldPose3d() {
        final Pose3d[] pose3ds = new Pose3d[notesOnField.size()];
        for (int i = 0; i < notesOnField.size(); i++) {
            final Vector2D notePosition2D = notesOnField.get(i).getFieldPosition();
            pose3ds[i] = new Pose3d(
                    new Translation3d(notePosition2D.getX(), notePosition2D.getY(), NoteOnField.noteHeight / 2),
                    new Rotation3d()
            );
        }
        return pose3ds;
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
