package frc.robot.parsers;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.parsers.json.IndexerConfJson;
import frc.robot.tagalong.FileUtils;
import java.io.File;

public class IndexerParser {
  public IndexerConfJson intakeConf;
  public RollerParser intakeSideParser;
  public RollerParser shooterSideParser;
  public RollerParser amptrapParser;


  public IndexerParser(File dir, String filename) {
    try {
      File intakeFile = new File(dir, filename);
      FileUtils.checkForFile(intakeFile);
      intakeConf = new ObjectMapper().readValue(intakeFile, IndexerConfJson.class);

      intakeSideParser = new RollerParser(
          new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/configs/indexer"),
          intakeConf.intakeFile
      );

      shooterSideParser = new RollerParser(
          new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/configs/indexer"),
          intakeConf.shooterFile
      );

      amptrapParser = new RollerParser(
          new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/configs/indexer"),
          intakeConf.ampTrapFile
      );

    } catch (Exception err) {
      System.err.println(err);
      System.exit(1);
    }
  }
}
