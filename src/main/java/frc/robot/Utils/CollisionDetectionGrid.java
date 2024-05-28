package frc.robot.Utils;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Utils.MathUtils.Vector2D;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

public class CollisionDetectionGrid {
    private boolean[][] grid = null;
    private double cellSize = 0, fieldLength = 0, fieldWidth = 0;

    public CollisionDetectionGrid() {
        try {
            File navGridFile = new File(Filesystem.getDeployDirectory(), "pathplanner/navgrid.json");
            StringBuilder fileContentBuilder = new StringBuilder();
            String line;
            BufferedReader br = new BufferedReader(new FileReader(navGridFile));
            while ((line = br.readLine()) != null) {
                fileContentBuilder.append(line);
            }

            String fileContent = fileContentBuilder.toString();
            JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

            cellSize = ((Number) json.get("nodeSizeMeters")).doubleValue();
            JSONArray grid = (JSONArray) json.get("grid");
            this.grid = new boolean[((JSONArray) grid.get(0)).size()][grid.size()];

            for (int row = 0; row < grid.size(); row++) {
                JSONArray rowArray = (JSONArray) grid.get(row);
                for (int col = 0; col < rowArray.size(); col++)
                    this.grid[col][row] = (boolean) rowArray.get(col);
            }

            JSONObject fieldSize = (JSONObject) json.get("field_size");
            fieldLength = ((Number) fieldSize.get("x")).doubleValue();
            fieldWidth = ((Number) fieldSize.get("y")).doubleValue();
        } catch (Exception ignored) {}
    }

    public boolean isInObstacle(Vector2D position) {
        final double x = Math.min(fieldLength, Math.max(0, position.getX())),
                y = Math.min(fieldWidth, Math.max(0, position.getY()));
        final int col = (int)(x / cellSize), row = (int) (y / cellSize);
        return grid[col][row];
    }

    /**
     * @return {boundedPosition, boundedVelocity}
     */
    public Vector2D[] applyCollisionDetection(Vector2D originalPosition, Vector2D originalVelocity) {
        // TODO finish this method
        if (!isInObstacle(originalPosition))
            return new Vector2D[] {originalPosition, originalVelocity};
        if (originalVelocity.getX() > 0) {
            if (originalPosition.getY() > 0) // particle moving up-right
                return null;
            if (originalPosition.getY() < 0) // particle moving down-right
                return null;
            return new Vector2D[] {originalPosition, originalVelocity}; // particle moving right
        }
        if (originalVelocity.getX() < 0) {
            if (originalPosition.getY() > 0) // particle moving up-left
                return null;
            if (originalPosition.getY() < 0) // particle moving down-left
                return null;
            return null; // particle moving left
        }

        if (originalPosition.getY() > 0) // particle moving up
            return null;
        if (originalPosition.getY() < 0) // particle moving down
            return null;
        return new Vector2D[] {originalPosition, originalVelocity}; // particle still
    }
}
