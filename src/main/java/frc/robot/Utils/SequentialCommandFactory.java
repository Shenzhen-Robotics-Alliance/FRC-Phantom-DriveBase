package frc.robot.Utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.AutoStagePrograms.FieldPositions;
import frc.robot.Modules.PositionReader.PositionEstimator;
import frc.robot.Modules.Chassis.SwerveBasedChassis;
import frc.robot.RobotCore;
import frc.robot.Utils.MathUtils.BezierCurve;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.SpeedCurves;
import frc.robot.Utils.MathUtils.Vector2D;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.io.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class SequentialCommandFactory {
    private final SwerveBasedChassis chassis;
    private final PositionEstimator positionEstimator;
    private final Vector2D robotStartingPosition;
    private final Rotation2D robotStartingRotation2D;

    public SequentialCommandFactory(RobotCore robotCore) {
        this(robotCore, new Vector2D(), new Rotation2D(0));
    }

    public SequentialCommandFactory(RobotCore robotCore, String firstPathName, Rotation2D robotStartingRotation2D) {
        this(robotCore, getRobotStartingPosition(firstPathName), robotStartingRotation2D);
    }

    public SequentialCommandFactory(RobotCore robotCore, Vector2D robotStartingPosition, Rotation2D robotStartingRotation2D) {
        this.chassis = robotCore.chassisModule;
        this.positionEstimator = robotCore.positionReader;
        this.maintainCurrentRotation = () -> new Rotation2D(positionEstimator.getRobotRotation());
        this.robotStartingPosition = robotStartingPosition;
        this.robotStartingRotation2D = robotStartingRotation2D;
    }

    private static final SequentialCommandSegment.InitiateCondition justGo = () -> true;
    private static final Runnable doNothing = () -> {};
    private static final SequentialCommandSegment.IsCompleteChecker weDoNotCareAboutIsItComplete = () -> true;
    private final SequentialCommandSegment.RotationFeeder maintainCurrentRotation;
    private static final SequentialCommandSegment.RotationFeeder weDoNotCareAboutRotation = () -> null;

    public SequentialCommandSegment calibratePositionEstimator() {
        return justDoIt(() -> {
            positionEstimator.setRobotRotation(robotStartingRotation2D.getRadian());
            positionEstimator.setRobotPosition(robotStartingPosition);
        });
    }

    public SequentialCommandSegment moveToPoint(Vector2D destination) {
        return moveToPoint(destination, doNothing, doNothing, doNothing);
    }

    public SequentialCommandSegment moveToPoint(Vector2D destination, Runnable beginning, Runnable periodic, Runnable ending) {
        return moveToPointIf(justGo, destination, beginning, periodic, ending);
    }

    public SequentialCommandSegment moveToPointIf(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D destination, Runnable beginning, Runnable periodic, Runnable ending) {
        return new SequentialCommandSegment(
                initiateCondition,
                () -> new BezierCurve(positionEstimator.getRobotPosition2D(), destination),
                beginning, periodic, ending,
                chassis::isCurrentTranslationalTaskFinished,
                maintainCurrentRotation, maintainCurrentRotation
        );
    }

    public SequentialCommandSegment moveToPointIf(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D destination, Runnable beginning, Runnable periodic, Runnable ending, Rotation2D endingRotation) {
        return new SequentialCommandSegment(
                initiateCondition,
                () -> new BezierCurve(positionEstimator.getRobotPosition2D(), destination),
                beginning, periodic, ending,
                chassis::isCurrentTranslationalTaskFinished,
                maintainCurrentRotation, () -> endingRotation
        );
    }

    public SequentialCommandSegment moveToPointAndStop(Vector2D destination) {
        return moveToPointAndStopIf(justGo, destination);
    }
    public SequentialCommandSegment moveToPointAndStop(Vector2D destination, Runnable beginning, Runnable periodic, Runnable ending) {
        return moveToPointAndStopIf(justGo, destination, beginning, periodic, ending);
    }

    public SequentialCommandSegment moveToPointAndStopIf(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D destination) {
        return moveToPointAndStopIf(initiateCondition, destination, doNothing, doNothing, doNothing);
    }
    public SequentialCommandSegment moveToPointAndStopIf(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D destination, Runnable beginning, Runnable periodic, Runnable ending) {
        return new SequentialCommandSegment(
                initiateCondition,
                () -> new BezierCurve(positionEstimator.getRobotPosition2D(), destination),
                beginning, periodic, ending,
                chassis::isCurrentTranslationalTaskFinished,
                maintainCurrentRotation, maintainCurrentRotation
        );
    }

    public SequentialCommandSegment moveFromPointToPoint(Vector2D startingPoint, Vector2D endingPoint) {
        return moveFromPointToPointIf(justGo, startingPoint, endingPoint, doNothing, doNothing, doNothing, maintainCurrentRotation, maintainCurrentRotation);
    }

    public SequentialCommandSegment moveFromPointToPoint(Vector2D startingPoint, Vector2D endingPoint, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToPoint(startingPoint, endingPoint, doNothing, doNothing, doNothing, startingRotation, endingRotation);
    }

    public SequentialCommandSegment moveFromPointToPoint(Vector2D startingPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToPointIf(justGo, startingPoint, endingPoint, beginning, periodic, ending, () -> startingRotation, () -> endingRotation);
    }

    public SequentialCommandSegment moveFromPointToPointIf(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D startingPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, SequentialCommandSegment.RotationFeeder startingRotationFeeder, SequentialCommandSegment.RotationFeeder endingRotationFeeder) {
        return new SequentialCommandSegment(
                initiateCondition,
                () -> new BezierCurve(startingPoint, endingPoint),
                beginning, periodic, ending,
                weDoNotCareAboutIsItComplete,
                startingRotationFeeder, endingRotationFeeder
        );
    }

    public SequentialCommandSegment moveFromPointToMidPointToPoint(Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint) {
        return moveFromPointToMidPointToPointIf(justGo, startingPoint, midPoint, endingPoint, doNothing, doNothing, doNothing, maintainCurrentRotation, maintainCurrentRotation);
    }

    public SequentialCommandSegment moveFromPointToMidPointToPoint(Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToMidPointToPoint(startingPoint, midPoint, endingPoint, doNothing, doNothing, doNothing, startingRotation, endingRotation);
    }

    public SequentialCommandSegment moveFromPointToMidPointToPoint(Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToMidPointToPointIf(justGo, startingPoint, midPoint, endingPoint, beginning, periodic, ending, () -> startingRotation, () -> endingRotation);
    }

    public SequentialCommandSegment moveFromPointToMidPointToPointIf(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, SequentialCommandSegment.RotationFeeder startingRotationFeeder, SequentialCommandSegment.RotationFeeder endingRotationFeeder) {
        return new SequentialCommandSegment(
                initiateCondition,
                () -> new BezierCurve(startingPoint, midPoint, endingPoint),
                beginning, periodic, ending,
                weDoNotCareAboutIsItComplete,
                startingRotationFeeder, endingRotationFeeder
        );
    }

    public SequentialCommandSegment moveFromPointToPointAndStop(Vector2D startingPoint, Vector2D endingPoint) {
        return moveFromPointToPointAndStopIf(justGo, startingPoint, endingPoint, doNothing, doNothing, doNothing, maintainCurrentRotation, maintainCurrentRotation);
    }

    public SequentialCommandSegment moveFromPointToPointAndStop(Vector2D startingPoint, Vector2D endingPoint, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToPointAndStop(startingPoint, endingPoint, doNothing, doNothing, doNothing, startingRotation, endingRotation);
    }

    public SequentialCommandSegment moveFromPointToPointAndStop(Vector2D startingPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToPointAndStopIf(justGo, startingPoint, endingPoint, beginning, periodic, ending, () -> startingRotation, () -> endingRotation);
    }

    public SequentialCommandSegment moveFromPointToPointAndStopIf(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D startingPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, SequentialCommandSegment.RotationFeeder startingRotationFeeder, SequentialCommandSegment.RotationFeeder endingRotationFeeder) {
        return new SequentialCommandSegment(
                initiateCondition,
                () -> new BezierCurve(startingPoint, endingPoint),
                beginning, periodic, ending,
                chassis::isCurrentTranslationalTaskFinished,
                startingRotationFeeder, endingRotationFeeder
        );
    }

    public SequentialCommandSegment moveFromPointToMidPointToPointAndStop(Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint) {
        return moveFromPointToMidPointToPointAndStopIf(justGo, startingPoint, midPoint, endingPoint, doNothing, doNothing, doNothing, maintainCurrentRotation, maintainCurrentRotation);
    }

    public SequentialCommandSegment moveFromPointToMidPointToPointAndStop(Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToMidPointToPointAndStop(startingPoint, midPoint, endingPoint, doNothing, doNothing, doNothing, startingRotation, endingRotation);
    }

    public SequentialCommandSegment moveFromPointToMidPointToPointAndStop(Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToMidPointToPointAndStopIf(justGo, startingPoint, midPoint, endingPoint, beginning, periodic, ending, () -> startingRotation, () -> endingRotation);
    }

    public SequentialCommandSegment moveFromPointToMidPointToPointAndStopIf(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, SequentialCommandSegment.RotationFeeder startingRotationFeeder, SequentialCommandSegment.RotationFeeder endingRotationFeeder) {
        return new SequentialCommandSegment(
                initiateCondition,
                () -> new BezierCurve(startingPoint, midPoint, endingPoint),
                beginning, periodic, ending,
                chassis::isCurrentTranslationalTaskFinished,
                startingRotationFeeder, endingRotationFeeder
        );
    }


    public SequentialCommandSegment faceDirection(Rotation2D direction){
        return faceDirection(direction, doNothing, doNothing, doNothing);
    }

    public SequentialCommandSegment faceDirection(Rotation2D direction, Runnable beginning, Runnable periodic, Runnable ending) {

        return new SequentialCommandSegment(
                justGo,
                () -> null,
                beginning, periodic, ending,
                chassis::isCurrentRotationalTaskFinished,
                positionEstimator::getRobotRotation2D,
                () -> direction
        );
    }

    public SequentialCommandSegment lockChassis() {
        return lockChassisFor(99999);
    }
    public SequentialCommandSegment lockChassisFor(long timeMillis) {
        final Timer timer = new Timer(); timer.start();
        return new SequentialCommandSegment(
                justGo,
                () -> null,
                () -> {
                    chassis.setChassisLocked(true, null);
                    timer.reset();
                },
                doNothing,
                doNothing,
                () -> timer.hasElapsed(timeMillis / 1000.0),
                weDoNotCareAboutRotation, weDoNotCareAboutRotation
        );
    }
    public SequentialCommandSegment lockChassisIfAndUntil(SequentialCommandSegment.InitiateCondition initiateCondition, SequentialCommandSegment.IsCompleteChecker isCompleteChecker) {
        return new SequentialCommandSegment(
                initiateCondition,
                () -> null,
                () -> chassis.setChassisLocked(true, null),
                doNothing,
                doNothing,
                isCompleteChecker,
                weDoNotCareAboutRotation, weDoNotCareAboutRotation
        );
    }

    public SequentialCommandSegment justDoIt(Runnable job) {
        return new SequentialCommandSegment(
                justGo,
                () -> null,
                job,
                doNothing,
                doNothing,
                weDoNotCareAboutIsItComplete,
                weDoNotCareAboutRotation, weDoNotCareAboutRotation
        );
    }

    public static Vector2D getRobotStartingPosition(String firstPathName) {
        try (BufferedReader br = new BufferedReader(new FileReader(new File(
                Filesystem.getDeployDirectory(), "pathplanner/paths/" + firstPathName + ".path")))) {
            StringBuilder fileContentBuilder = new StringBuilder();
            String line;
            while ((line = br.readLine()) != null) {
                fileContentBuilder.append(line);
            }
            String fileContent = fileContentBuilder.toString();
            JSONObject pathJson = (JSONObject) new JSONParser().parse(fileContent);
            JSONArray waypointsJson = (JSONArray) pathJson.get("waypoints");

            JSONObject firstPoint = (JSONObject) waypointsJson.get(0);

            return pointFromJson((JSONObject) firstPoint.get("anchor"));
        } catch (FileNotFoundException e) {
            throw new RuntimeException("Cannot Find Path File: " + firstPathName + " From Deploy Directory: " + Filesystem.getDeployDirectory());
        } catch (IOException e) {
            throw new RuntimeException("IO Error While Reading File: " + firstPathName);
        } catch (ParseException e) {
            throw new RuntimeException("Error Occurred While Processing JSON Path File: " + firstPathName);
        }
    }

    public SequentialCommandSegment followSingleCurve(String pathName, int index, Rotation2D facingRotation) {
        return followSingleCurve(pathName, index, facingRotation, doNothing, doNothing, doNothing);
    }

    public SequentialCommandSegment followSingleCurve(String pathName, int index, Rotation2D facingRotation, Runnable beginning, Runnable periodic, Runnable ending) {
        final List<BezierCurve> curves = getBezierCurvesFromPathFile(pathName);
        return new SequentialCommandSegment(
                () -> true,
                () -> curves.get(index),
                beginning, periodic, ending,
                () -> true,
                positionEstimator::getRobotRotation2D, () -> FieldPositions.toActualRotation(facingRotation),
                SpeedCurves.originalSpeed,1
        );
    }

    /** all rotations are in red alliance, will be automatically converted if blue */
    public SequentialCommandSegment[] followPathFacing(String pathName, Rotation2D facingRotation) {
        return followPathFacing(pathName, facingRotation, doNothing, doNothing, doNothing);
    }
    public SequentialCommandSegment[] followPathFacing(String pathName, Rotation2D facingRotation, Runnable beginning, Runnable periodic, Runnable ending) {
        final Rotation2D[] rotationTargets = new Rotation2D[getBezierCurvesFromPathFile(pathName).size()];
        Arrays.fill(rotationTargets, facingRotation);
        return followPath(pathName, rotationTargets, beginning, periodic, ending);
    }

    public SequentialCommandSegment[] followPath(String pathName) {
        return followPath(pathName, doNothing, doNothing, doNothing);
    }


    public SequentialCommandSegment[] followPath(String pathName, Runnable beginning, Runnable periodic, Runnable ending) {
        return followPath(pathName, new Rotation2D[getBezierCurvesFromPathFile(pathName).size()+1], beginning, periodic, ending);
    }

    public SequentialCommandSegment[] followPath(String pathName, Rotation2D[] robotRotationTargets, Runnable beginning, Runnable periodic, Runnable ending) {
        final List<BezierCurve> curves = getBezierCurvesFromPathFile(pathName);
        final SequentialCommandSegment[] commandSegments = new SequentialCommandSegment[curves.size()];
        System.out.println("curves.size(): " + curves.size());
        System.out.println("rotation targets size: " + robotRotationTargets.length);
        if (curves.size() != robotRotationTargets.length)
            throw new IllegalStateException("Error While Scheduling Follow Path Command: " + pathName + ". Rotational targets length (" + robotRotationTargets.length + ") do not match pathplanner checkpoints number (" + curves.size() + ")");

        if (curves.size() == 1)
            return new SequentialCommandSegment[] {
                    new SequentialCommandSegment(
                            () -> true,
                            () -> curves.get(0),
                            beginning, periodic, ending,
                            chassis::isCurrentTranslationalTaskFinished,
                            positionEstimator::getRobotRotation2D, () -> FieldPositions.toActualRotation(robotRotationTargets[0]),
                            SpeedCurves.easeInOut,1
                    )
            };

        commandSegments[0] = new SequentialCommandSegment(
                () -> true,
                () -> curves.get(0),
                beginning, periodic, doNothing,
                () -> true,
                positionEstimator::getRobotRotation2D, () -> FieldPositions.toActualRotation(robotRotationTargets[0]),
                SpeedCurves.easeIn,1
        );

        for (int i = 1; i < curves.size()-1; i++) {
            final BezierCurve curve = curves.get(i);
            final Rotation2D startingRotation = robotRotationTargets[i-1], endingRotation = robotRotationTargets[i];
            commandSegments[i] = new SequentialCommandSegment(
                    () -> true,
                    () -> curve,
                    doNothing, periodic, doNothing,
                    () -> true,
                    () -> FieldPositions.toActualRotation(startingRotation), () -> FieldPositions.toActualRotation(endingRotation),
                    SpeedCurves.originalSpeed,1
            );
        }

        commandSegments[commandSegments.length-1] = new SequentialCommandSegment(
                () -> true,
                () -> curves.get(curves.size()-1),
                doNothing, periodic, ending,
                chassis::isCurrentTranslationalTaskFinished,
                () -> FieldPositions.toActualRotation(robotRotationTargets[robotRotationTargets.length-2]), () -> FieldPositions.toActualRotation(robotRotationTargets[robotRotationTargets.length-1]),
                SpeedCurves.originalSpeed,1
        );

        return commandSegments;
    }

    public static List<BezierCurve> getBezierCurvesFromPathFile(String pathName) {
        try (BufferedReader br = new BufferedReader(new FileReader(new File(
                Filesystem.getDeployDirectory(), "pathplanner/paths/" + pathName + ".path")))) {
            StringBuilder fileContentBuilder = new StringBuilder();
            String line;
            while ((line = br.readLine()) != null) fileContentBuilder.append(line);

            String fileContent = fileContentBuilder.toString();
            JSONObject pathJson = (JSONObject) new JSONParser().parse(fileContent);
            JSONArray waypointsJson = (JSONArray) pathJson.get("waypoints");

            List<BezierCurve> curves = new ArrayList<>();
            for (int i = 0; i < waypointsJson.size() - 1; i++) {
                JSONObject point = (JSONObject) waypointsJson.get(i),
                        nextPoint = (JSONObject) waypointsJson.get(i+1);
                curves.add(new BezierCurve(
                        pointFromJson((JSONObject) point.get("anchor")),
                        pointFromJson((JSONObject) point.get("nextControl")),
                        pointFromJson((JSONObject) nextPoint.get("prevControl")),
                        pointFromJson((JSONObject) nextPoint.get("anchor"))
                ));
            }
            return curves;
        } catch (FileNotFoundException e) {
            throw new RuntimeException("Cannot Find Path File: " + pathName + " From Deploy Directory: " + Filesystem.getDeployDirectory());
        } catch (IOException e) {
            throw new RuntimeException("IO Error While Reading File: " + pathName);
        } catch (ParseException e) {
            throw new RuntimeException("Error Occurred While Processing JSON Path File: " + pathName);
        }
    }

    /**
     * converts a point from pathplanner to vector2D
     * pathplanner is always in red alliance
     * converts to red / blue alliance according to driver-station, defaults to red
     * */
    private static Vector2D pointFromJson(JSONObject pointJson) {
        final double x = ((Number) pointJson.get("x")).doubleValue();
        final double y = ((Number) pointJson.get("y")).doubleValue();

        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);

        final double fieldHeight = 8.21, fieldWidth = 16.54;
//        return switch (alliance) {
//            case Red -> new Vector2D(new double[]{y - fieldHeight / 2, fieldWidth - x});
//            case Blue -> new Vector2D(new double[]{fieldHeight / 2 - y, x});
//        };
        return new Vector2D(new double[] {
                (alliance == DriverStation.Alliance.Blue ? -1:1) * (y - fieldHeight / 2),
                fieldWidth - x
        });
    }
}
