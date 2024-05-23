package frc.robot.Utils.MechanismControllers;

import frc.robot.Utils.EasyShuffleBoard;

/**
 * controls the speed of fly wheel
 * speed is also positive
 * */
public class FlyWheelSpeedController implements MechanismController {
    private SimpleFeedForwardSpeedController simpleFeedForwardSpeedController;
    private FlyWheelSpeedControllerProfile profile;
    private static final double nanoToSec = 1_000_000_000.0;
    /**
     * if the difference between the current desired velocity and the new desired velocity is within this range, we just skip to new desired velocity right away
     * notice that this value is represented in the percentage of maximum speed
     * */
    private static final double jumpToDesiredSpeedMarginInMaximumSpeed = 0.1;

    private double speedWhenTaskStarted, desiredSpeed;
    private long taskStartTimeNano;
    public FlyWheelSpeedController(FlyWheelSpeedControllerProfile profile) {
        this.profile = profile;
        this.simpleFeedForwardSpeedController = new SimpleFeedForwardSpeedController(profile);
        this.desiredSpeed = 0;
        startNewSpeedControlTask( 0);
    }

    public void setProfile(FlyWheelSpeedControllerProfile profile) {
        this.profile = profile;
        this.simpleFeedForwardSpeedController = new SimpleFeedForwardSpeedController(profile);
    }

    public void setDesiredSpeed(double newDesiredSpeed) {
        // v = at, t = v / a
        if (Math.abs(newDesiredSpeed - desiredSpeed) < jumpToDesiredSpeedMarginInMaximumSpeed * profile.maximumSpeed)
            this.desiredSpeed = newDesiredSpeed;
        else
            startNewSpeedControlTask(newDesiredSpeed);
    }

    public double getCorrectionPower(double currentSpeed) {
        if (Math.abs(getCurrentTargetSpeedWithLERP() / profile.maximumSpeed) < 0.02) return 0;
        final double correctionSpeed = simpleFeedForwardSpeedController.getSpeedControlPower(
                currentSpeed / profile.maximumSpeed,
                getCurrentTargetSpeedWithLERP() / profile.maximumSpeed
        );
        EasyShuffleBoard.putNumber("shooter", "flywheel controller current target speed", getCurrentTargetSpeedWithLERP());
        return correctionSpeed; // do not go negative power
    }

    private void startNewSpeedControlTask(double newDesiredSpeed) {
        this.speedWhenTaskStarted = desiredSpeed;
        this.desiredSpeed = newDesiredSpeed;
        this.taskStartTimeNano = System.nanoTime();
    }

    private double getCurrentTargetSpeedWithLERP() {
        final double speedDifferenceBetweenTaskStartAndEnd = desiredSpeed - speedWhenTaskStarted,
                speedDifferenceMaximumMagnitude = Math.abs(speedDifferenceBetweenTaskStartAndEnd),
                timeSinceTaskStarted = (System.nanoTime() - taskStartTimeNano) / nanoToSec,
                speedDifferenceReached = Math.min(timeSinceTaskStarted * profile.maximumAcceleration, speedDifferenceMaximumMagnitude),
                currentTargetSpeed = speedWhenTaskStarted + Math.copySign(speedDifferenceReached, speedDifferenceBetweenTaskStartAndEnd);
        return Math.max(Math.min(profile.maximumSpeed, currentTargetSpeed),-profile.maximumSpeed);
    }

    @Override
    public double getMotorPower(double mechanismVelocity, double mechanismPosition) {
        return this.getCorrectionPower(mechanismVelocity);
    }

    public static class FlyWheelSpeedControllerProfile extends SimpleFeedForwardSpeedController.SimpleFeedForwardControllerProfile {
        public final double maximumSpeed, maximumAcceleration;
        public FlyWheelSpeedControllerProfile(double proportionGain, double feedForwardGain, double frictionGain, double feedForwardDelay, double maximumSpeed, double timeNeededToAccelerateToMaxSpeed) {
            super(proportionGain, feedForwardGain, frictionGain, feedForwardDelay);
            this.maximumSpeed = maximumSpeed;
            this.maximumAcceleration = maximumSpeed/timeNeededToAccelerateToMaxSpeed;
        }
    }

    public FlyWheelSpeedControllerProfile getProfile() {
        return profile;
    }
}
