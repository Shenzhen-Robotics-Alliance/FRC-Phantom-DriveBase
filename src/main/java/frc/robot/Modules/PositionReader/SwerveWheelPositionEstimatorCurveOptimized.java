package frc.robot.Modules.PositionReader;

import frc.robot.Drivers.IMUs.SimpleGyro;
import frc.robot.Modules.Chassis.SwerveWheel;
import frc.robot.Utils.EasyShuffleBoard;
import frc.robot.Utils.MathUtils.AngleUtils;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;

public class SwerveWheelPositionEstimatorCurveOptimized extends SwerveWheelPositionEstimator {
    /**
     * swerve wheel position reader,
     *
     * @param swerveWheels
     * @param gyro
     */
    public SwerveWheelPositionEstimatorCurveOptimized(SwerveWheel[] swerveWheels, SimpleGyro gyro) {
        super(swerveWheels, gyro);
    }

    private final double[] wheelDriveEncoderReadings = new double[swerveWheels.length];
    private final double[] wheelSteerEncoderReadings = new double[swerveWheels.length];

    @Override
    public void onReset() {
        for (int wheelID = 0; wheelID < swerveWheels.length; wheelID++) {
            wheelDriveEncoderReadings[wheelID] = swerveWheels[wheelID].getWheelDrivingEncoderValue();
            wheelSteerEncoderReadings[wheelID] = swerveWheels[wheelID].getSteerHeading();
        }
        super.onReset();
    }

    @Override
    protected void periodic(double dt) {
        for (int wheelID = 0; wheelID < swerveWheels.length; wheelID++) {
            /* calculate velocity */
            Vector2D wheelVelocity = swerveWheels[wheelID].getModuleVelocity2D(); // gain the velocity of each module, in vector
            wheelVelocity = wheelVelocity.multiplyBy(new Rotation2D(gyro.getYaw())); // apply rotation according to the imu

            /* calculate displacement using curve-optimization */
            final double newDriveEncoderReading = swerveWheels[wheelID].getWheelDrivingEncoderValue(),
                    distance = newDriveEncoderReading - wheelDriveEncoderReadings[wheelID],
                    newSteerEncoderReading = swerveWheels[wheelID].getSteerHeading();
            final Vector2D displacement = calculateDisplacementWithCurve(
                    wheelSteerEncoderReadings[wheelID],
                    newSteerEncoderReading,
                    distance
            );
            wheelPositions[wheelID] = wheelPositions[wheelID].addBy(displacement);
            wheelDriveEncoderReadings[wheelID] = newDriveEncoderReading;
            wheelSteerEncoderReadings[wheelID] = newSteerEncoderReading;

            wheelAccelerations[wheelID] = wheelVelocity
                    .addBy(wheelVelocities[wheelID].multiplyBy(-1))
                    .multiplyBy(1.0f/dt); // apply derivative to time

            wheelVelocities[wheelID] = wheelVelocity; // keep a copy of the velocity
        }

        EasyShuffleBoard.putNumber("chassis", "new position estimator (x)", getRobotPosition2D().getX());
        EasyShuffleBoard.putNumber("chassis", "new position estimator (y)", getRobotPosition2D().getY());
    }


    /**
     * calculates, using a circular curve as approximation, the displacement done by a swerve module in a given period
     * @param startingRotation rotation of the swerve at the beginning of the period, in radian and in reference to the field, 0 is to the right and counter-clockwise is positive
     * @param endingRotation rotation of the swerve at the end of the period, in radian and in reference to the field, 0 is to the right and counter-clockwise is positive
     * @param distance the distance, in any unit X, that the wheel have travelled during the period (or the length of the curve)
     * @return the displacement during the period, in reference to the field and in the same unit X as param "distance"
     * */
    private static Vector2D calculateDisplacementWithCurve(double startingRotation, double endingRotation, double distance) {
        final double theta = AngleUtils.getActualDifference(startingRotation, endingRotation),
                x = getXCoe(theta) * distance,
                y = getYCoe(theta) * distance;
        final Vector2D displacementInReferenceToStartingRotation = new Vector2D(new double[] {x, y});
        return displacementInReferenceToStartingRotation.multiplyBy(new Rotation2D(startingRotation));
    }

    /**
     * x-displacement = k * distance, where k = sin(theta)/theta
     * @param theta angle theta
     * @return coefficient k
     * */
    private static double getXCoe(double theta) {
        if (Math.abs(theta) < 1E-4) return 1; // the l'Hôpital's rule
        return Math.sin(theta) / theta;
    }

    /**
     * y-displacement = k * distance, where k = (1-cos(theta))/theta
     * @param theta angle theta
     * @return coefficient k
     * */
    private static double getYCoe(double theta) {
        if (Math.abs(theta) < 1E-4) return 0; // the l'Hôpital's rule
        return (1-Math.cos(theta)) / theta;
    }
}
