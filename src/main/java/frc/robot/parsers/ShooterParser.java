package frc.robot.parsers;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.parsers.json.ShooterConfJson;
import frc.robot.tagalong.FileUtils;
import java.io.File;

public class ShooterParser {
  public ShooterConfJson shooterConf;
  public FlywheelParser flywheel1Parser;
  public FlywheelParser flywheel2Parser;
  public PivotParser pivotParser;
  public RollerParser rollerParser;

  public ShooterParser(File dir, String filename) {
    try {
      File shooterFile = new File(dir, filename);
      FileUtils.checkForFile(shooterFile);
      shooterConf = new ObjectMapper().readValue(shooterFile, ShooterConfJson.class);

      flywheel1Parser = new FlywheelParser(
          new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/configs/shooter"),
          shooterConf.flywheelLeftFile
      );

      flywheel2Parser = new FlywheelParser(
          new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/configs/shooter"),
          shooterConf.flywheelRightFile
      );

      pivotParser = new PivotParser(
          new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/configs/shooter"),
          shooterConf.pivotFile
      );

    } catch (Exception err) {
      System.err.println(err);
      System.exit(1);
    }
  }
}
