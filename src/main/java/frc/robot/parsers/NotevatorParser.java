package frc.robot.parsers;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.parsers.json.NotevatorConfJson;
import frc.robot.tagalong.FileUtils;
import java.io.File;

public class NotevatorParser {
  public NotevatorConfJson notevatorConf;
  public ElevatorParser elevatorParser;

  public NotevatorParser(File dir, String filename) {
    try {
      File notevatorFile = new File(dir, filename);
      FileUtils.checkForFile(notevatorFile);
      notevatorConf = new ObjectMapper().readValue(notevatorFile, NotevatorConfJson.class);

      elevatorParser = new ElevatorParser(
          new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/configs/notevator"),
          notevatorConf.elevatorFile
      );

    } catch (Exception err) {
      System.err.println(err);
      System.exit(1);
    }
  }
}
