package frc.robot.parsers;

import java.io.File;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.parsers.json.ClimberConfJson;
import frc.robot.tagalong.FileUtils;

public class ClimberParser {
  public ClimberConfJson climberConf;
  public ElevatorParser elevatorRightParser;
  public ElevatorParser elevatorLeftParser;

  public ClimberParser(File dir, String filename) {
    try {
      File ampTrapFile = new File(dir, filename);
      FileUtils.checkForFile(ampTrapFile);
      climberConf = new ObjectMapper().readValue(ampTrapFile, ClimberConfJson.class);

      elevatorRightParser = new ElevatorParser(
          new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/configs/climber"),
          climberConf.elevatorRightFile
      );
      elevatorLeftParser = new ElevatorParser(
          new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/configs/climber"),
          climberConf.elevatorLeftFile
      );

    } catch (Exception err) {
      System.err.println(err);
      System.exit(1);
    }
  }
}
