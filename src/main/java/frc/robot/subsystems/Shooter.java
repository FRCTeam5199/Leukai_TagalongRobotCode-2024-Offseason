package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.parsers.ShooterParser;
import frc.robot.parsers.json.ShooterConfJson;
import frc.robot.subsystems.minor.TagalongDualMotorFlywheel;
import frc.robot.subsystems.minor.TagalongPivot;
import frc.robot.subsystems.minor.TagalongRoller;
import frc.robot.tagalong.FlywheelAugment;
import frc.robot.tagalong.PivotAugment;
import frc.robot.tagalong.RollerAugment;
import frc.robot.tagalong.TagalongSubsystemBase;

public class Shooter extends TagalongSubsystemBase implements PivotAugment, FlywheelAugment {

    public Shooter(Object parser) {
        super(parser);
        //TODO Auto-generated constructor stub
    }

    public final ShooterParser _shooterParser;
    public final ShooterConfJson _shooterConf;

    private final TagalongDualMotorFlywheel _flywheel;
    private final TagalongPivot _pivot;

    @Override
    public TagalongDualMotorFlywheel getFlywheel() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getFlywheel'");
    }

    @Override
    public TagalongDualMotorFlywheel getFlywheel(int i) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getFlywheel'");
    }

    @Override
    public TagalongPivot getPivot() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPivot'");
    }

    @Override
    public TagalongPivot getPivot(int i) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPivot'");
    }


    public void onEnable(){
        _flywheel.onEnable();
        _pivot.onEnable();
    }

    public Shooter(String filePath) {
        this(filePath == null ? null : new ShooterParser(Filesystem.getDeployDirectory(), filePath));
    }

     public Shooter(ShooterParser parser) {
    super(parser);
    _shooterParser = parser;

    if (_configuredDisable) {
      _shooterConf = null;
      _pivotUnsafeMinRot = 0.0;
      _pivotUnsafeMaxRot = 0.0;
      _flywheel = new TagalongDualMotorFlywheel(null);
      _pivot = new TagalongPivot(null);
      return;
    }

    _shooterConf = _shooterParser.shooterConf;
    _flywheel = new TagalongDualMotorFlywheel(_shooterParser.flywheelParser);
    _pivot = new TagalongPivot(_shooterParser.pivotParser);
    _pivotUnsafeMinRot = _shooterConf.pivotUnsafePositionalLimits.getMinRot();
    _pivotUnsafeMaxRot = _shooterConf.pivotUnsafePositionalLimits.getMaxRot();

    int counter = 0;
    while (!checkInitStatus() && counter < 100) {
      System.out.println("Waiting for Shooter");
    }

    if (counter >= 100) {
      System.out.println("failed to init Shooter");
    }

    _chuteBreakBeam = new DigitalInput(_shooterConf.breakBeamChannel);

    configShuffleboard();
  }

    
}
