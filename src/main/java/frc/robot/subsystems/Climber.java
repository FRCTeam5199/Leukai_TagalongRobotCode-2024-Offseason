package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.parsers.ClimberParser;
import frc.robot.parsers.json.ClimberConfJson;
import frc.robot.subsystems.minor.TagalongElevator;
import frc.robot.tagalong.ElevatorAugment;
import frc.robot.tagalong.TagalongSubsystemBase;

public class Climber extends TagalongSubsystemBase implements ElevatorAugment {
  public static final class ElevatorConstants {
    public static final int ELEVATOR_ID = 0;

    public static final double ELEVATOR_ZEROING_SPEED_MPS = -0.06;
    public static final double ELEVATOR_PREP_SPEED_MPS = -0.01;
    public static final double ELEVATOR_ZEROING_STALL_TOLERANCE = 50;
    public static final int ELEVATOR_ZEROING_STALL_LOOPS = 6;

    public static final double ELEVATOR_AMP_MPS = 1.0;
    public static final double ELEVATOR_TRAP_MPS = 1.0;
  }

  private final TagalongElevator _elevatorLeft;
  private final TagalongElevator _elevatorRight;
  public final ClimberParser _climberParser;
  public final ClimberConfJson _climberConf;

  /* -------- Logging: utilities and configs -------- */
  // private final ClimberIOTalonFX _io;
  // private final ClimberIOInputsAutoLogged _inputs = new ClimberIOInputsAutoLogged();

  @Override
  public TagalongElevator getElevator() {
    return _elevatorRight;
  }

  @Override
  public TagalongElevator getElevator(int i) {
    switch (i) {
      case 1:
        return _elevatorLeft;
      case 0:
      default:
        return _elevatorRight;
    }
  }

  public Climber(String filePath) {
    this(filePath == null ? null : new ClimberParser(Filesystem.getDeployDirectory(), filePath));
  }

  public Climber(ClimberParser parser) {
    super(parser);
    _climberParser = parser;

    if (_configuredDisable) {
      // _io = null;
      _climberConf = null;
      _elevatorRight = new TagalongElevator(null);
      _elevatorLeft = new TagalongElevator(null);
      return;
    }

    _climberConf = _climberParser.climberConf;
    _elevatorRight = new TagalongElevator(parser.elevatorParser);
    _elevatorLeft = new TagalongElevator(parser.elevatorParser);

    int counter = 0;
    while (!checkInitStatus() && counter < 100) {
      System.out.println("Waiting for climber");
    }
    if (counter >= 100) {
      System.out.println("failed to init");
    }

    // THIS HAS TO GO LAST
    // _io = new ClimberIOTalonFX(this);
    configShuffleboard();
  }

  public boolean isReadyToClimb() {
    return _isSubsystemDisabled ? true : false;
  }

  public boolean isClimbed() {
    return _isSubsystemDisabled ? true : false;
  }

  @Override
  public void onEnable() {
    if (_isSubsystemDisabled) {
      return;
    }
    _elevatorLeft.onEnable();
    _elevatorRight.onEnable();
    // for testing
    // _elevator.getElevatorMotor().setControl(
    //     new VelocityVoltage(0.000001).withFeedForward(_elevator._elevatorFF.ks)
    // );
    // System.out.println("ks " + _elevator._elevatorFF.ks);
  }

  public void onDisable() {
    if (_isSubsystemDisabled) {
      return;
    }
    _elevatorRight.onDisable();
    _elevatorLeft.onDisable();
  }

  @Override
  public void periodic() {
    if (_isSubsystemDisabled) {
      return;
    }
    _elevatorRight.periodic();
    _elevatorLeft.periodic();

    // Logging
    // _io.updateInputs(_inputs);
    // Logger.processInputs("Climber", _inputs);
    updateShuffleboard();
  }

  @Override
  public void disabledPeriodic() {
    _elevatorRight.disabledPeriodic();
    _elevatorLeft.disabledPeriodic();
  }

  @Override
  public void simulationInit() {
    _elevatorRight.simulationInit();
    _elevatorLeft.simulationInit();
  }

  @Override
  public void simulationPeriodic() {
    _elevatorRight.simulationPeriodic();
    _elevatorLeft.simulationPeriodic();
  }

  @Override
  public void updateShuffleboard() {
    _elevatorRight.updateShuffleboard();
    _elevatorLeft.updateShuffleboard();
  }

  @Override
  public void configShuffleboard() {
    _elevatorRight.configShuffleboard();
    _elevatorLeft.configShuffleboard();
  }

  // TODO IMPLEMENT
  @Override
  public boolean checkInitStatus() {
    return super.checkInitStatus() && _elevatorRight.checkInitStatus() && _elevatorLeft.checkInitStatus();
  }
}
