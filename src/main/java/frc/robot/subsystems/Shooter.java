package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.parsers.ShooterParser;
import frc.robot.subsystems.minor.TagalongDualMotorFlywheel;
import frc.robot.subsystems.minor.TagalongPivot;
import frc.robot.tagalong.FlywheelAugment;
import frc.robot.tagalong.PivotAugment;
import frc.robot.tagalong.TagalongSubsystemBase;

public class Shooter extends TagalongSubsystemBase implements PivotAugment, FlywheelAugment {
    public Shooter(String filePath) {
        this(filePath == null ? null : new ShooterParser(Filesystem.getDeployDirectory(), filePath));
    }

    public Shooter(ShooterParser parser) {
        super(parser);
        shooterParser = parser;
        if (isShooterSubsystemDisabled) {
          shooter1 = new TagalongDualMotorFlywheel(null);
          shooter2 = new TagalongDualMotorFlywheel(null);
          arm = new TagalongPivot(null);
          return;
        }
        shooter1 = new TagalongDualMotorFlywheel(shooterParser.flywheel1Parser);
        shooter2 = new TagalongDualMotorFlywheel(shooterParser.flywheel2Parser);
        arm = new TagalongPivot(shooterParser.pivotParser);
    }

    public final ShooterParser shooterParser;

    private final TagalongDualMotorFlywheel shooter1;
    private final TagalongDualMotorFlywheel shooter2;
    private final TagalongPivot arm;
    private boolean isShooterSubsystemDisabled = false;

    @Override
    public TagalongDualMotorFlywheel getFlywheel() {
      return shooter1;
    }

    @Override
    public TagalongDualMotorFlywheel getFlywheel(int i) {
        switch (i) {
          case 0:
            return shooter1;
          case 1:
            return shooter2;
          default:
            return shooter1;
        }
    }

    @Override
    public TagalongPivot getPivot() {
        return arm;
    }

    @Override
    public TagalongPivot getPivot(int i) {
      return arm;
    }

    @Override
    public void onEnable(){
        shooter1.onEnable();
        shooter2.onEnable();
        arm.onEnable();
    }

    @Override
    public void onDisable() {
      if (_isSubsystemDisabled) {
        return;
      }
      shooter1.onDisable();
      shooter2.onDisable();
      arm.onDisable();
    }
  
    @Override
    public void periodic() {
      if (_isSubsystemDisabled) {
        return;
      }
      shooter1.periodic();
      shooter2.periodic();
      arm.periodic();
    }
  
    @Override
    public void disabledPeriodic() {
      shooter1.disabledPeriodic();
      shooter2.disabledPeriodic();
      arm.disabledPeriodic();
    }
  
    @Override
    public void simulationInit() {
      shooter1.simulationInit();
      shooter2.simulationInit();
      arm.simulationInit();
    }
  
    @Override
    public void simulationPeriodic() {
      shooter1.simulationPeriodic();
      shooter2.simulationPeriodic();
      arm.simulationPeriodic();
    }
  
    @Override
    public void updateShuffleboard() {
      shooter1.updateShuffleboard();
      shooter2.updateShuffleboard();
      arm.updateShuffleboard();
    }
  
    @Override
    public void configShuffleboard() {
      shooter1.configShuffleboard();
      shooter2.configShuffleboard();
      arm.configShuffleboard();
    }
  
    @Override
    public boolean checkInitStatus() {
      return super.checkInitStatus() && shooter1.checkInitStatus() && shooter2.checkInitStatus()
          && arm.checkInitStatus();
    }
}
