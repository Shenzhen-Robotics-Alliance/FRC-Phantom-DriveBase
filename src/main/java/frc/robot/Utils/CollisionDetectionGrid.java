package frc.robot.Utils;

import edu.wpi.first.wpilibj.Filesystem;
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

    public boolean isInObstacle(double x, double y) {
        final int col = (int)(x / cellSize), row = (int) (y / cellSize);
        return 0 <= col && col < grid.length && 0 <= row && row < grid[0].length && grid[col][row];
    }
}
