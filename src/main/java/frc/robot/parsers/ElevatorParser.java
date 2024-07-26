package frc.robot.parsers;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.parsers.json.ElevatorConfJson;
import frc.robot.tagalong.FileUtils;
import java.io.File;

public class ElevatorParser {
  public ElevatorConfJson elevatorConf;
  public ElevatorParser elevatorParser;
  public RollerParser rollerParser;

  public ElevatorParser(File dir, String filename) {
    try {
      File elevatorFile = new File(dir, filename);
      FileUtils.checkForFile(elevatorFile);
      elevatorConf = new ObjectMapper().readValue(elevatorFile, ElevatorConfJson.class);

      elevatorParser = new ElevatorParser(
          new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/configs/climber"),
          elevatorConf.elevatorFile
      );
      rollerParser = new RollerParser(
          new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/configs/climber"),
          elevatorConf.rollerFile
      );

    } catch (Exception err) {
      System.err.println(err);
      System.exit(1);
    }
  }
}
