package frc.robot.Utils.Tests;

import frc.robot.Utils.EasyDataFlow;
import frc.robot.Utils.MathUtils.AngleUtils;
import frc.robot.Utils.MathUtils.LookUpTable;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.PilotController;
import frc.robot.Utils.RobotConfigReader;

public class DataFlowTest implements SimpleRobotTest {
    final PilotController pilotController;

    public DataFlowTest(RobotConfigReader robotConfig) {
        this.pilotController = new PilotController(robotConfig, "control-XBOX");
    }

    @Override
    public void testStart() {
        previousTimeMillis = System.currentTimeMillis();
        pos = new Vector2D();
        vel = new Vector2D();
    }

    Vector2D pos, vel;
    double rotation = 0;
    final double maxChassisSpeed = 5.2, maxLinearAcceleration = 8, maxCentripetalAcceleration = 10, friction = 8, maxAngularVelocity = Math.toRadians(180);
    long previousTimeMillis;
    @Override
    public void testPeriodic() {
        final double dt = (System.currentTimeMillis() - previousTimeMillis) / 1000.0f;
        pilotController.update();
        final Vector2D desiredVelocity = pilotController.getTranslationalStickValue().multiplyBy(maxChassisSpeed); // TODO here, rotated the desired velocity

        double desiredSpeed = desiredVelocity.getMagnitude(),
                desiredMovingDirection = desiredVelocity.getHeading();
        if (Math.abs(AngleUtils.getActualDifference(vel.getHeading(), desiredMovingDirection)) > Math.toRadians(90)) {
            desiredMovingDirection = AngleUtils.simplifyAngle(desiredMovingDirection + Math.PI);
            desiredSpeed *= -1;
        }
        final double speedDifference = desiredSpeed - vel.getMagnitude(),
                linearAccelerationConstrain = speedDifference > 0 ?
                        LookUpTable.linearInterpretation(0, maxLinearAcceleration, maxChassisSpeed, 0, vel.getMagnitude())
                        : -LookUpTable.linearInterpretation(0, friction, maxChassisSpeed, 0, desiredSpeed),
                speedChange = linearAccelerationConstrain * dt,
                minStep = Math.abs(linearAccelerationConstrain) * 0.1,
                newSpeed = Math.abs(speedDifference) < minStep ?
                        desiredSpeed : vel.getMagnitude() + speedChange,

                desiredMotionDirection = desiredVelocity.getMagnitude() / maxChassisSpeed < 0.03 ? vel.getHeading() : desiredMovingDirection,
                headingDifference = AngleUtils.getActualDifference(vel.getHeading(), desiredMotionDirection),
                angularVelocityConstrain = maxCentripetalAcceleration / (1+vel.getMagnitude()),
                headingChange = Math.copySign(angularVelocityConstrain * dt, headingDifference),
                headingMinStep = angularVelocityConstrain * 0.1,
                newHeading =
                        Math.abs(headingDifference) < headingMinStep || (vel.getMagnitude() / maxChassisSpeed < 0.1) ?
                                desiredMotionDirection : AngleUtils.simplifyAngle(vel.getHeading() + headingChange);

        vel = new Vector2D(newHeading, Math.min(maxChassisSpeed, newSpeed));
        if (desiredVelocity.getMagnitude() == 0 && vel.getMagnitude() / maxChassisSpeed < 0.1)
            vel = new Vector2D();
        EasyDataFlow.putNumber("test", "dt", dt);
        EasyDataFlow.putNumber("test", "speed difference", speedDifference);
        EasyDataFlow.putNumber("test", "speed change", speedChange);
        EasyDataFlow.putNumber("test", "pilot input mag", desiredVelocity.getMagnitude());
        EasyDataFlow.putNumber("test",  "new speed", newSpeed);
        pos = pos.addBy(vel.multiplyBy(dt));
        rotation += pilotController.getRotationalStickValue() * dt * maxAngularVelocity;
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
