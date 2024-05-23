package frc.robot.Utils;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Utils.MathUtils.Transformation2D;
import frc.robot.Utils.MathUtils.Vector2D;

public class PilotController {
    private final GenericHID rawController;
    private final boolean[] buttonsOnHold;
    private final boolean[] buttonsOnPress;
    private final boolean[] buttonsOnRelease;
    private final int xAxisPortID, yAxisPortID, zAxisPortID;
    private final double zSensitivity,
            stickThresholdWhenOtherAxisUnused, stickThresholdWhenOtherAxisFull,
            translationalAxisExponent, rotationalAxisExponent;
    private final Transformation2D sensitivityTransformation;
    private Vector2D translationalStickValue = new Vector2D();
    private double rotationalStickValue = 0;

    /**
     * creates a pilot controller
     * @param rawController the instance of the controller pad
     * @param xAxisPortID the id of the x-axis
     * @param yAxisPortID the id of the y-axis
     * @param zAxisPortID the id of the z-axis
     * @param xSensitivity the sensitivity of the x-axis, from -1~1
     * @param ySensitivity the sensitivity of the y-axis, from -1~1
     * @param zSensitivity the sensitivity of the z-axis, from -1~1
     * @param stickThresholdWhenOtherAxisUnused the threshold for an axis when the other axis is zero
     * @param stickThresholdWhenOtherAxisFull the threshold for an axis when the other axis is 1 (or -1)
     * @param translationalAxisExponent the exponent of the rotation input, 1~2
     * @param rotationalAxisExponent the exponent of the translational input, 1~2
     */
    public PilotController(
            GenericHID rawController,
            int xAxisPortID, int yAxisPortID, int zAxisPortID,
            double xSensitivity, double ySensitivity, double zSensitivity,
            double stickThresholdWhenOtherAxisUnused, double stickThresholdWhenOtherAxisFull,
            double translationalAxisExponent, double rotationalAxisExponent) {
        this.rawController = rawController;
        this.xAxisPortID = xAxisPortID;
        this.yAxisPortID = yAxisPortID;
        this.zAxisPortID = zAxisPortID;

        this.sensitivityTransformation = new Transformation2D(
                new Vector2D(new double[] {xSensitivity, 0}),
                new Vector2D(new double[] {0, ySensitivity})
        );

        this.zSensitivity = zSensitivity;
        this.stickThresholdWhenOtherAxisUnused = stickThresholdWhenOtherAxisUnused;
        this.stickThresholdWhenOtherAxisFull = stickThresholdWhenOtherAxisFull;
        this.translationalAxisExponent = translationalAxisExponent;
        this.rotationalAxisExponent = rotationalAxisExponent;

        this.buttonsOnHold = new boolean[rawController.getButtonCount()];
        this.buttonsOnPress = new boolean[rawController.getButtonCount()];
        this.buttonsOnRelease = new boolean[rawController.getButtonCount()];
    }

    public PilotController(RobotConfigReader robotConfig, String configDomainName) {
        this(
                new GenericHID((int)robotConfig.getConfig(configDomainName, "controllerPort")),
                (int)robotConfig.getConfig(configDomainName, "xAxisPort"),
                (int)robotConfig.getConfig(configDomainName, "yAxisPort"),
                (int)robotConfig.getConfig(configDomainName, "zAxisPort"),
                robotConfig.getConfig(configDomainName, "pilotControllerXAxisSensitivity") / 100.0f,
                robotConfig.getConfig(configDomainName, "pilotControllerYAxisSensitivity") / 100.0f,
                robotConfig.getConfig(configDomainName, "pilotControllerZAxisSensitivity") / 100.0f,
                robotConfig.getConfig(configDomainName, "pilotStickThresholdWhenOtherAxisUnused") / 100.0f,
                robotConfig.getConfig(configDomainName, "pilotStickThresholdWhenOtherAxisFull") / 100.0f,
                robotConfig.getConfig(configDomainName, "pilotTranslationalAxisExponent"),
                robotConfig.getConfig(configDomainName, "pilotRotationalAxisExponent")
        );
    }

    public void updateTranslationalStickValue() {
        final Vector2D rawUnlimitedTranslationalStickValue = new Vector2D(new double[] {
                rawController.getRawAxis(xAxisPortID), rawController.getRawAxis(yAxisPortID)}),
                rawTranslationalStickValue = applyLimit(rawUnlimitedTranslationalStickValue),
                translationalStickValue = applySmartThreshold(rawTranslationalStickValue);

        final double translationalStickMagnitudeRaw = translationalStickValue.getMagnitude(),
                translationalStickMagnitude = applyExponent(translationalStickMagnitudeRaw, translationalAxisExponent);

        this.translationalStickValue = new Vector2D(translationalStickValue.getHeading(), translationalStickMagnitude)
                .multiplyBy(sensitivityTransformation);
    }

    public Vector2D getTranslationalStickValue() {
        return translationalStickValue;
    }

    public double getRawAxis(int port) {
        return rawController.getRawAxis(port);
    }

    public void updateRotationalStickValue() {
        this.rotationalStickValue = applyExponent(
                applyThreshold(rawController.getRawAxis(zAxisPortID), stickThresholdWhenOtherAxisUnused),
                rotationalAxisExponent) * zSensitivity;
    }

    public double getRotationalStickValue() {
        return rotationalStickValue;
    }

    private Vector2D applyLimit(Vector2D unlimitedStickValue) {
        return new Vector2D(unlimitedStickValue.getHeading(), Math.min(1, unlimitedStickValue.getMagnitude()));
    }

    private Vector2D applySmartThreshold(Vector2D rawStickValue) {
        final double xThreshold = getThreshold(rawStickValue.getY());
        final double yThreshold = getThreshold(rawStickValue.getX());

        return new Vector2D(new double[] {
                applyThreshold(rawStickValue.getX(), xThreshold),
                applyThreshold(rawStickValue.getY(), yThreshold)
        });
    }

    private double getThreshold(double otherAxisValue) {
        return stickThresholdWhenOtherAxisUnused + Math.abs(otherAxisValue) * (stickThresholdWhenOtherAxisFull - stickThresholdWhenOtherAxisUnused);
    }
    private static double applyThreshold(double value, double threshold) {
        return Math.abs(value) >= threshold ? value: 0;
    }

    private static double applyExponent(double value, double exponent) {
        return Math.copySign(Math.pow(Math.abs(value), exponent), value);
    }

    public void update() {
        for (int keyPort = 1; keyPort < buttonsOnHold.length; keyPort++) {
            final boolean currentStatus = rawController.getRawButton(keyPort);
            this.buttonsOnPress[keyPort] = !buttonsOnHold[keyPort] && currentStatus;
            this.buttonsOnRelease[keyPort] = buttonsOnHold[keyPort] && !currentStatus;
            this.buttonsOnHold[keyPort] = currentStatus;
        }

        updateTranslationalStickValue();
        updateRotationalStickValue();
    }

    public boolean keyOnHold(int keyPort) {
        try {
            return this.buttonsOnHold[keyPort];
        } catch (IndexOutOfBoundsException consumed) {
            return false;
        }
    }

    public boolean keyOnPress(int keyPort) {
        try {
            return this.buttonsOnPress[keyPort];
        } catch (IndexOutOfBoundsException consumed) {
            return false;
        }
    }

    public boolean keyOnRelease(int keyPort) {
        try {
            return this.buttonsOnRelease[keyPort];
        } catch (IndexOutOfBoundsException consumed) {
            return false;
        }
    }
}
