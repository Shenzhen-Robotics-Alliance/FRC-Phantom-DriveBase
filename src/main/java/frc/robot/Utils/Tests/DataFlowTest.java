package frc.robot.Utils.Tests;

import edu.wpi.first.units.Angle;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Utils.EasyDataFlow;
import frc.robot.Utils.MathUtils.AngleUtils;
import frc.robot.Utils.MathUtils.LookUpTable;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;

public class DataFlowTest implements SimpleRobotTest {
    @Override
    public void testStart() {
        previousTimeMillis = System.currentTimeMillis();
        pos = new Vector2D();
        vel = new Vector2D();
    }

    Vector2D pos, vel;
    double rotation = 0;
    final double maxVelocity = 5, maxAcceleration = 10, maxAccelerationAtFullSpeed = 4, friction = 8, maxAngularVelocity = Math.toRadians(180);
    long previousTimeMillis;
    @Override
    public void testPeriodic() {
        final XboxController xboxController = new XboxController(0);
        final double dt = (System.currentTimeMillis() - previousTimeMillis) / 1000.0f;
        final double oldSpeed = vel.getMagnitude();
        final Vector2D desiredMotion = new Vector2D(new double[] {xboxController.getLeftX(), -xboxController.getLeftY()}),
                accByMotor = desiredMotion.multiplyBy(LookUpTable.linearInterpretation(0, maxAcceleration, maxVelocity, maxAccelerationAtFullSpeed, oldSpeed) + friction);

        vel = vel.addBy(accByMotor.multiplyBy(dt));
        final Vector2D centripetalForce = new Vector2D()
        final double velChangeDueToFriction = friction * dt;
        if (vel.getMagnitude() < velChangeDueToFriction)
            vel = new Vector2D();
        else
            vel = new Vector2D(vel.getHeading(), vel.getMagnitude() - velChangeDueToFriction);

        final double linearAccConstrain = LookUpTable.linearInterpretation(0, maxAcceleration, maxVelocity, 0, oldSpeed),
                newSpeedConstrain = Math.min(maxVelocity, oldSpeed + linearAccConstrain * dt);
        vel = new Vector2D(vel.getHeading(), Math.min(newSpeedConstrain, vel.getMagnitude()));

        pos = pos.addBy(vel.multiplyBy(dt));
        rotation += xboxController.getRightX() * dt * maxAngularVelocity;
        EasyDataFlow.putPosition("robot pos", pos, new Rotation2D(rotation));

//        EasyDataFlow.putSwerveState(
//                "chassis",
//                new Vector2D(new double[] {0,1}),
//                new Vector2D(new double[] {0,0}),
//                new Vector2D(new double[] {0,0}),
//                new Vector2D(new double[] {0,0}),
//                new Rotation2D(Math.toRadians(90))
//        );

//        EasyDataFlow.putPositionArray(
//                "position 1",
//                new Vector2D[] {new Vector2D(new double[] {3, 3}), new Vector2D(new double[] {2.5, 2.5})},
//                new Rotation2D[] {new Rotation2D(0), new Rotation2D(Math.PI)});

        previousTimeMillis = System.currentTimeMillis();
    }
}
