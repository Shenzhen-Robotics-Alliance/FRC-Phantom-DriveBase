package frc.robot.Utils.Tests;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Modules.PositionReader.RobotFieldPositionEstimator;
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
        previousTime = Timer.getFPGATimestamp();
        pos = new Vector2D();
        vel = new Vector2D();
    }

    Vector2D pos, vel;
    double rotation = 0;
    final double
            /* the maximum chassis speed when the chassis is moving in a straight line */
            maxChassisSpeed = 4.6,
            /* the maximum chassis speed when the chassis is doing sharp turn */
            maxChassisSpeedAtSharpTurn = 2.4,
            /*  */
            maxLinearAcceleration = 8,
            maxCentripetalAcceleration = 12,
            friction = 16,
            maxAngularVelocity = Math.toRadians(270);
    double previousTime = Timer.getFPGATimestamp();
    @Override
    public void testPeriodic() {
        final double dt = (Timer.getFPGATimestamp() - previousTime);
        pilotController.update();
        final Rotation2D pilotFacing = RobotFieldPositionEstimator.toActualRobotRotation(new Rotation2D(Math.toRadians(270)));
        final Vector2D desiredVelocity = pilotController.getTranslationalStickValue().multiplyBy(maxChassisSpeed).multiplyBy(pilotFacing);

        double desiredSpeed = desiredVelocity.getMagnitude(),
                pilotStickDirection = desiredVelocity.getHeading();
        final boolean turnAround =
                Math.abs(AngleUtils.getActualDifference(vel.getHeading(), pilotStickDirection)) > Math.toRadians(120)
                && vel.getMagnitude() != 0
                && desiredSpeed != 0;
        if (turnAround) {
            pilotStickDirection = AngleUtils.simplifyAngle(pilotStickDirection + Math.PI);
            desiredSpeed *= -1;
        }
        final double
                minStepTime = 0.05,
                speedDifference = desiredSpeed - vel.getMagnitude(),
                linearAccelerationConstrain = speedDifference > 0 ?
                        LookUpTable.linearInterpretationWithBounding(0, maxLinearAcceleration, maxChassisSpeed, 0, vel.getMagnitude())
                        : -LookUpTable.linearInterpretationWithBounding(0, friction, maxChassisSpeed, 0, desiredSpeed),
                speedChange = linearAccelerationConstrain * dt,
                linearSpeedMinStep = Math.abs(linearAccelerationConstrain) * minStepTime,
                newSpeed = Math.abs(speedDifference) < linearSpeedMinStep ?
                        Math.abs(desiredSpeed) : vel.getMagnitude() + speedChange,

                desiredMotionDirection = desiredVelocity.getMagnitude() / maxChassisSpeed < 0.03 ? vel.getHeading() : pilotStickDirection,
                headingDifference = AngleUtils.getActualDifference(vel.getHeading(), desiredMotionDirection),
                angularVelocityConstrain = maxCentripetalAcceleration / (1+vel.getMagnitude()),
                headingChange = Math.copySign(angularVelocityConstrain * dt, headingDifference),
                headingMinStep = angularVelocityConstrain * minStepTime,
                newHeading =
                        Math.abs(headingDifference) < headingMinStep || (vel.getMagnitude() / maxChassisSpeed < 0.1) ?
                                desiredMotionDirection : AngleUtils.simplifyAngle(vel.getHeading() + headingChange);
        vel = new Vector2D(newHeading, newSpeed);

        final double currentMaxChassisSpeed =
                LookUpTable.linearInterpretationWithBounding(
                        0,
                        maxChassisSpeed,
                        Math.toRadians(90),
                        maxChassisSpeedAtSharpTurn,
                        Math.abs(AngleUtils.getActualDifference(vel.getHeading(), desiredVelocity.getHeading()))
                );
        vel = new Vector2D(newHeading, Math.min(currentMaxChassisSpeed, vel.getMagnitude()));

        if ((desiredVelocity.getMagnitude() == 0 || turnAround)
                && vel.getMagnitude() / maxChassisSpeed < 0.1)
            vel = new Vector2D();
        EasyDataFlow.putNumber("test", "dt", dt);
        EasyDataFlow.putNumber("test", "speed difference", speedDifference);
        EasyDataFlow.putNumber("test", "speed change", speedChange);
        EasyDataFlow.putNumber("test", "pilot input mag", desiredVelocity.getMagnitude());
        EasyDataFlow.putNumber("test",  "new speed", newSpeed);
        EasyDataFlow.putNumber("test", "chassis max spd", currentMaxChassisSpeed);
        EasyDataFlow.putNumber("test", "pilot stick dir", Math.toDegrees(pilotStickDirection));
        EasyDataFlow.putNumber("test", "pilot stick dir", Math.toDegrees(pilotStickDirection));
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

        previousTime = Timer.getFPGATimestamp();
    }
}
