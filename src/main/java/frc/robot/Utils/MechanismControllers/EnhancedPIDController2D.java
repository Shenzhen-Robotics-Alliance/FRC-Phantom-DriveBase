package frc.robot.Utils.MechanismControllers;

import frc.robot.Utils.MathUtils.Vector2D;

public class EnhancedPIDController2D {
    private final EnhancedPIDController xController, yController;
    private final EnhancedPIDController.PIDProfile xConfig, yConfig;
    private final double xMinPowerToMove, yMinPowerToMove;
    private long previousTimeMillis;

    public EnhancedPIDController2D(EnhancedPIDController.PIDProfile config) {
        this(config, config);
    }

    public EnhancedPIDController2D(EnhancedPIDController.PIDProfile xConfig, EnhancedPIDController.PIDProfile yConfig) {
        this.xController = new EnhancedPIDController(xConfig);
        this.yController = new EnhancedPIDController(yConfig);
        this.xConfig = xConfig;
        this.yConfig = yConfig;
        xMinPowerToMove = xConfig.getMinPowerToMove();
        yMinPowerToMove = yConfig.getMinPowerToMove();
        this.previousTimeMillis = System.currentTimeMillis();
    }

    public void startNewTask(Task2D task) {
        xController.startNewTask(task.xTask);
        yController.startNewTask(task.yTask);
    }

    public Vector2D getCorrectionPower(Vector2D currentPosition) {
        double dt = (System.currentTimeMillis() - previousTimeMillis) / 1000.0f;
        double xCorrectionPower = this.xController.getMotorPower(currentPosition.getX(), dt);
        double yCorrectionPower = this.yController.getMotorPower(currentPosition.getY(), dt);
        this.previousTimeMillis = System.currentTimeMillis();
        return new Vector2D(new double[]{xCorrectionPower, yCorrectionPower});
    }

    public Vector2D getCorrectionPower(Vector2D currentPosition, Vector2D currentVelocity) {
        double dt = (System.currentTimeMillis() - previousTimeMillis) / 1000.0f;
        double xCorrectionPower = this.xController.getMotorPower(currentPosition.getX(), currentVelocity.getX(), dt);
        double yCorrectionPower = this.yController.getMotorPower(currentPosition.getY(), currentVelocity.getY(), dt);

        /* if correction power along the one axis is big enough, we discard friction compensation of the other axis */
        if (Math.abs(yCorrectionPower) > 2 * yMinPowerToMove)
            xConfig.setMinPowerToMove(xMinPowerToMove / 3);
        else
            xConfig.setMinPowerToMove(xMinPowerToMove);
        if (Math.abs(xCorrectionPower) > 2 * xMinPowerToMove)
            yConfig.setMinPowerToMove(yMinPowerToMove / 3);
        else
            yConfig.setMinPowerToMove(yMinPowerToMove);
        this.previousTimeMillis = System.currentTimeMillis();
        return new Vector2D(new double[]{xCorrectionPower, yCorrectionPower});
    }

    public static final class Task2D {
        public final EnhancedPIDController.Task xTask, yTask;
        public Task2D(EnhancedPIDController.Task.TaskType taskType, Vector2D value) {
            this.xTask = new EnhancedPIDController.Task(taskType, value.getX());
            this.yTask = new EnhancedPIDController.Task(taskType, value.getY());
        }
    }

    private static boolean almostEqual(double a, double b) {
        return Math.abs(a - b) < 0.01;
    }
}
