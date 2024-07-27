package frc.robot.parsers;

import com.fasterxml.jackson.databind.ObjectMapper;
import frc.robot.parsers.json.utils.Flywheel2MConfJson;
import frc.robot.tagalong.FileUtils;
import java.io.File;

public class Flywheel2MParser {
  public Flywheel2MConfJson flywheelConf;

  public Flywheel2MParser(File dir, String filename) {
    try {
      File flywheelFile = new File(dir, filename);
      FileUtils.checkForFile(flywheelFile);
      flywheelConf = new ObjectMapper().readValue(flywheelFile, Flywheel2MConfJson.class);

    } catch (Exception err) {
      System.err.println(err);
      System.exit(1);
    }
  }
}
