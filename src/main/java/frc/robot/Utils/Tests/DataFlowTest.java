package frc.robot.Utils.Tests;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Utils.EasyDataFlow;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import org.littletonrobotics.junction.Logger;

public class DataFlowTest implements SimpleRobotTest {
    @Override
    public void testStart() {
        previousTimeMillis = System.currentTimeMillis();
        pos = new Vector2D();
        vel = new Vector2D();
    }

    Vector2D pos, vel;
    double rotation = 0;
    final double maxVelocity = 10, maxAcceleration = 20, friction = 10, maxAngularVelocity = Math.toRadians(180);
    long previousTimeMillis;
    @Override
    public void testPeriodic() {
        final XboxController xboxController = new XboxController(0);
        final double dt = (System.currentTimeMillis() - previousTimeMillis) / 1000.0f;
        final Vector2D acc = new Vector2D(new double[] {xboxController.getLeftX(), -xboxController.getLeftY()}).multiplyBy((1-vel.getMagnitude()/maxVelocity) * maxAcceleration);
        vel = vel.addBy(acc.multiplyBy(dt));
        vel = new Vector2D(vel.getHeading(), Math.min(maxVelocity, vel.getMagnitude()));
        final double velChangeDueToFriction=  friction * dt;
        if (vel.getMagnitude() < velChangeDueToFriction)
            vel = new Vector2D();
        else
            vel = new Vector2D(vel.getHeading(), vel.getMagnitude() - velChangeDueToFriction);

        pos = pos.addBy(vel.multiplyBy(dt));
        rotation += xboxController.getRightX() * dt * maxAngularVelocity;
        EasyDataFlow.putPosition("robot pos", pos, new Rotation2D(rotation));

        EasyDataFlow.putSwerveState(
                "chassis",
                new Vector2D(new double[] {0,1}),
                new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}),
                new Vector2D(new double[] {0,0}),
                new Rotation2D(Math.toRadians(90))
        );

//        EasyDataFlow.putPositionArray(
//                "position 1",
//                new Vector2D[] {new Vector2D(new double[] {3, 3}), new Vector2D(new double[] {2.5, 2.5})},
//                new Rotation2D[] {new Rotation2D(0), new Rotation2D(Math.PI)});

        previousTimeMillis = System.currentTimeMillis();
    }
}
