package frc.robot.parsers;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.parsers.json.AmpTrapConfJson;
import frc.robot.tagalong.FileUtils;
import java.io.File;

public class AmpTrapParser {
  public AmpTrapConfJson ampTrapConf;
  public ElevatorParser elevatorParser;
  public RollerParser rollerParser;

  public AmpTrapParser(File dir, String filename) {
    try {
      File ampTrapFile = new File(dir, filename);
      FileUtils.checkForFile(ampTrapFile);
      ampTrapConf = new ObjectMapper().readValue(ampTrapFile, AmpTrapConfJson.class);

      elevatorParser = new ElevatorParser(
          new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/configs/ampTrap"),
          ampTrapConf.elevatorFile
      );

      rollerParser = new RollerParser(
          new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/configs/ampTrap"),
          ampTrapConf.elevatorFile
      );

    } catch (Exception err) {
      System.err.println(err);
      System.exit(1);
    }
  }
}
