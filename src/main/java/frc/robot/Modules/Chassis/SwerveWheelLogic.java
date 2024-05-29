package frc.robot.Modules.Chassis;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Modules.RobotModuleBase;
import frc.robot.Utils.ChassisUnit;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Transformation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import frc.robot.Utils.RobotConfigReader;
import frc.robot.Utils.RobotModuleOperatorMarker;

public abstract class SwerveWheelLogic extends RobotModuleBase {
    protected final RobotConfigReader robotConfig;
    protected final int swerveWheelID;
    private final Vector2D wheelPositionVector;
    private final Vector2D rotationDirectionVector;

    /** the desired position to drive */
    protected double commandedHeading;
    /** the set motor speed to drive */
    protected double targetedSpeed;
    /** the actual, applied drive power */
    protected double drivePower;
    /** the amount of time since last operation */
    private Timer lastOperationTimer = new Timer();
    /** whether to go to default position, or stick the pilot's order */

    /**
     * whether to reverse the wheel
     * when the wheel is reversed, the steer turns 180-degree to the desired heading
     * and the motor turns reversely
     * we do it whenever this reduces the distance for the wheel to turn
     */
    protected boolean reverseWheel;
    protected boolean locked;
    protected SwerveWheelLogic(int swerveWheelID, RobotConfigReader robotConfig, Vector2D wheelPositionVector) {
        super("swerve module " + swerveWheelID);
        this.robotConfig = robotConfig;
        this.swerveWheelID = swerveWheelID;
        this.wheelPositionVector = wheelPositionVector;

        /*
         * calculate the direction of rotation motion of the robot in reference to the
         * wheel
         */
        final Transformation2D rotate90DegCounterWiseTransformation = new Rotation2D(Math.toRadians(90));
        final Vector2D rotationDirectionVectorRaw = wheelPositionVector.multiplyBy(rotate90DegCounterWiseTransformation);
        this.rotationDirectionVector = new Vector2D(rotationDirectionVectorRaw.getHeading(), 0.8); // the rotation vector always have magnitude 1
    }

    @Override
    protected void periodic(double dt) {

    }

    public double decideSwerveFacingDirection() {
        /** whether the robot is asked to move */
        boolean robotRequiredToMove = Math.abs(targetedSpeed) > lowestUsageSpeed;
        /* if there is an action */
        if (robotRequiredToMove)
            lastOperationTimer.reset(); // reset the timer for last operation

        /* proceed x-formation if required to lock */
        if (locked)
            return getWheelInstalledLocationVector().getHeading();

        /* the actual targeted steer heading decided by the system */
        if (lastOperationTimer.hasElapsed(maxUnusedTime))
            return defaultPosition;

        return commandedHeading;
    }

    public double getDecidedDrivePower() {
        return drivePower;
    }

    /* <-- configurations --> */
    /**
     * the maximum amount of time unused, after which the wheel will just go back to
     * default position
     */
    private double maxUnusedTime;
    /** the default position of the wheel */
    private double defaultPosition;
    /**
     * the slowest speed to be thought as being used, anything lower than this is
     * called "unused"
     */
    private double lowestUsageSpeed;


    @Override
    public void updateConfigs() {
        this.maxUnusedTime = robotConfig.getConfig("chassis/maxUnusedTime");
        this.defaultPosition = Math.toRadians(robotConfig.getConfig("chassis/defaultPosition"));
        this.lowestUsageSpeed = robotConfig.getConfig("chassis/minUsageSpeed");
    }

    @Override
    public void init() {

    }

    @Override
    public void onReset() {
        this.commandedHeading = defaultPosition;
        lastOperationTimer.start();
        this.commandedHeading = this.targetedSpeed = this.drivePower = 0;
    }

    /**
     * set the driving parameter of the current steer module, given the raw params
     * of the wheel, the wheel will drive exactly the way you tell it if you call
     * this method
     *
     * @param heading              the direction to drive, in radian. 0 is to the
     *                             front and
     *                             positive is counter-clockwise
     * @param speed                the motor speed to drive, in percent output. from
     *                             0~1
     * @param operator the robot service that is sending command to the
     *                             module, use this
     * @return the set motor speed
     */
    public double drive(double heading, double speed, RobotModuleOperatorMarker operator) {
        if (!isOwner(operator))
            return 0;
        final boolean inUse = speed > lowestUsageSpeed;
        this.commandedHeading = inUse ? heading : commandedHeading;
        this.targetedSpeed = inUse ? speed : 0;
        return speed;
    }

    /**
     * set the driving parameter of the current steer wheel, given a 2d vector with
     * x and y sitting in between0~1, representing the desired motion of the current
     * wheel
     *
     * @param desiredWheelMotion   a 2d vector with x and y sitting in the bound
     *                             0~1, representing the motion of the robot
     * @param operator the robot service that is sending the commands,
     *                             use "this"
     * @return the magnitude of the given drive vector
     */
    public double drive(Vector2D desiredWheelMotion, RobotModuleOperatorMarker operator) {
        return drive(desiredWheelMotion.getHeading(), desiredWheelMotion.getMagnitude(), operator);
    }

    /**
     * set the motion of the wheel, given the robot's desired motion and this
     * function will process them automatically
     *
     * @param desiredRobotMotion        a 2d vector with x and y in the range of
     *                                  0~1, representing the desired motion of the
     *                                  actual robot
     * @param desiredRobotRotationSpeed the desired angular speed of the robot,
     *                                  positive is counter-clockwise as in geometry
     * @return the amount of motor power of the wheel needed to do the operation, from 0~1
     * note that some operations might make motors go over full power, this will tear the chassis apart, so in chassis module we need to avoid that
     */
    public double drive(Vector2D desiredRobotMotion, double desiredRobotRotationSpeed, RobotModuleOperatorMarker operator) {
        Vector2D rotationVector = rotationDirectionVector.multiplyBy(desiredRobotRotationSpeed);
        return drive(desiredRobotMotion.addBy(rotationVector), operator);
    }

    public void setWheelLocked(boolean locked, RobotModuleOperatorMarker operatorMarker) {
        if (!isOwner(operatorMarker))
            return;
        this.locked = locked;
    }

    /** gets the position of the wheel on the robot */
    public Vector2D getWheelInstalledLocationVector() {
        return wheelPositionVector;
    }

    /** gets the direction vector of rotation of this wheel, which 90-deg counter-clockwise to the wheel */
    public Vector2D getRotationDirectionVector() {
        return rotationDirectionVector;
    }

    public double getWheelDrivingEncoderValue() {
        return getWheelDrivingEncoderValue(ChassisUnit.METER);
    }
    public abstract double getWheelDrivingEncoderValue(ChassisUnit unit);

    public Rotation2D getSteerFacing() {
        return new Rotation2D(getSteerHeading());
    }
    public abstract double getSteerHeading();
    public Vector2D getModuleVelocity2D() {
        return getModuleVelocity2D(ChassisUnit.METER); // default value
    }
    public abstract Vector2D getModuleVelocity2D(ChassisUnit unit);
}
