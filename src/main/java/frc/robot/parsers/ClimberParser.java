package frc.robot.parsers;

import java.io.File;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.parsers.json.ClimberConfJson;
import frc.robot.tagalong.FileUtils;

public class ClimberParser {
  public ClimberConfJson climberConf;
  public ElevatorParser elevatorParserRight;
  public ElevatorParser elevatorParserLeft;

  public ClimberParser(File dir, String filename) {
    try {
      File notevatorFile = new File(dir, filename);
      FileUtils.checkForFile(notevatorFile);
      climberConf = new ObjectMapper().readValue(notevatorFile, ClimberConfJson.class);

      elevatorParserRight = new ElevatorParser(
          new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/configs/climber"),
          climberConf.elevatorFile
      );
      elevatorParserLeft = new ElevatorParser(
          new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/configs/climber"),
          climberConf.elevatorFile
      );

    } catch (Exception err) {
      System.err.println(err);
      System.exit(1);
    }
  }
}
