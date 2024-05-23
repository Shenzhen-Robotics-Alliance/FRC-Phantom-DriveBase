package frc.robot.Utils.MechanismControllers;

import edu.wpi.first.math.MathUtil;
import frc.robot.Utils.MathUtils.AngleUtils;
import frc.robot.Utils.MathUtils.LookUpTable;

public class SimpleArmController implements MechanismController {
    private final double maxPowerWhenMovingUp, maxPowerWhenMovingDown, errorStartDecelerate, powerNeededToMoveUp, powerNeededToMoveDown, errorTolerance;
    private final boolean isEncoderAbsolute;
    public double desiredPosition = 0;
    public SimpleArmController(double maxPowerWhenMovingUp, double maxPowerWhenMovingDown, double errorStartDecelerate, double powerNeededToMoveUp, double powerNeededToMoveDown, double errorTolerance) {
        this(maxPowerWhenMovingUp, maxPowerWhenMovingDown, errorStartDecelerate, powerNeededToMoveUp, powerNeededToMoveDown, errorTolerance, true);
    }

    public SimpleArmController(double maxPowerWhenMovingUp, double maxPowerWhenMovingDown, double errorStartDecelerate, double powerNeededToMoveUp, double powerNeededToMoveDown, double errorTolerance, boolean isEncoderAbsolute) {
        this.maxPowerWhenMovingUp = maxPowerWhenMovingUp;
        this.maxPowerWhenMovingDown = maxPowerWhenMovingDown;
        this.errorStartDecelerate = errorStartDecelerate;
        this.powerNeededToMoveUp = powerNeededToMoveUp;
        this.powerNeededToMoveDown = powerNeededToMoveDown;
        this.errorTolerance = errorTolerance;
        this.isEncoderAbsolute = isEncoderAbsolute;
    }


    @Override
    public double getMotorPower(double mechanismVelocity, double mechanismPosition) {
        final double error = isEncoderAbsolute ? AngleUtils.getActualDifference(mechanismPosition, desiredPosition) : desiredPosition - mechanismPosition;
        if (Math.abs(error) < errorTolerance)
            return 0;

        final double correctionPower;
        if (error < 0) {
            correctionPower = -LookUpTable.linearInterpretation(errorTolerance, powerNeededToMoveUp, errorStartDecelerate, maxPowerWhenMovingUp, Math.abs(error));
        } else
            correctionPower = LookUpTable.linearInterpretation(errorTolerance, powerNeededToMoveDown, errorStartDecelerate, maxPowerWhenMovingDown, Math.abs(error));
        return MathUtil.clamp(correctionPower, -maxPowerWhenMovingUp, maxPowerWhenMovingDown);
    }
}
