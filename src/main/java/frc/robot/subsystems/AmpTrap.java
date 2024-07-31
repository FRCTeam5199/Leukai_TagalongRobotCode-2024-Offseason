package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.parsers.AmpTrapParser;
import frc.robot.parsers.json.AmpTrapConfJson;
import frc.robot.subsystems.minor.TagalongDualMotorElevator;
import frc.robot.subsystems.minor.TagalongElevator;
import frc.robot.tagalong.ElevatorAugment;
import frc.robot.tagalong.TagalongSubsystemBase;

public class AmpTrap extends TagalongSubsystemBase implements ElevatorAugment {
  public static final class ElevatorConstants {
    public static final int ELEVATOR_ID = 0;

    public static final double ELEVATOR_ZEROING_SPEED_MPS = -0.06;
    public static final double ELEVATOR_PREP_SPEED_MPS = -0.01;
    public static final double ELEVATOR_ZEROING_STALL_TOLERANCE = 50;
    public static final int ELEVATOR_ZEROING_STALL_LOOPS = 6;

    public static final double ELEVATOR_AMP_MPS = 1.0;
    public static final double ELEVATOR_TRAP_MPS = 1.0;
  }

  public static final class RollerConstants { public static final double ROLLER_AMP_SHOT = 50.0; }

  private final TagalongDualMotorElevator elevator;
  public final AmpTrapParser ampTrapParser;
  public final AmpTrapConfJson ampTrapConf;

  /* -------- Logging: utilities and configs -------- */
  // private final ClimberIOTalonFX _io;
  // private final ClimberIOInputsAutoLogged _inputs = new ClimberIOInputsAutoLogged();

  @Override
  public TagalongDualMotorElevator getElevator() {
    return elevator;
  }

    private static AmpTrap elevatorSubsystem;

    public AmpTrap(String filePath) {
        this(filePath == null ? null : new AmpTrapParser(Filesystem.getDeployDirectory(), filePath));
    }

    public AmpTrap(AmpTrapParser parser) {
        super(parser);
        ampTrapParser = parser;

        if (_configuredDisable) {
            // _io = null;
            ampTrapConf = null;
            elevator = new TagalongDualMotorElevator(null);
            return;
        }

        ampTrapConf = ampTrapParser.ampTrapConf;
        elevator = new TagalongDualMotorElevator(ampTrapParser.elevatorParser);

        int counter = 0;
        while (!checkInitStatus() && counter < 100) {
            System.out.println("Waiting for elevator");
        }
        if (counter >= 100) {
            System.out.println("failed to init");
        }

        // THIS HAS TO GO LAST
        // _io = new ClimberIOTalonFX(this);
        configShuffleboard();
    }

    public static AmpTrap getInstance() {
        if (elevatorSubsystem == null) {
            elevatorSubsystem = new AmpTrap("configs/ampTrap/ampTrapConf.json");
        }
        return elevatorSubsystem;
    }

    /* -------- Logging: utilities and configs -------- */
    // private final ClimberIOTalonFX _io;
    // private final ClimberIOInputsAutoLogged _inputs = new ClimberIOInputsAutoLogged();


    @Override
    public TagalongElevator getElevator(int i) {
        return elevator;
    }

    public boolean isReadyToElevate() {
        return _isSubsystemDisabled ? true : false;
    }

    public boolean isElevated() {
        return _isSubsystemDisabled ? true : false;
    }

    @Override
    public void onEnable() {
        if (_isSubsystemDisabled) {
            return;
        }
        elevator.onEnable();
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
        elevator.onDisable();
    }

    @Override
    public void periodic() {
        if (_isSubsystemDisabled) {
            return;
        }
        elevator.periodic();

        // Logging
        // _io.updateInputs(_inputs);
        // Logger.processInputs("Climber", _inputs);

        updateShuffleboard();
    }

    @Override
    public void disabledPeriodic() {
        elevator.disabledPeriodic();
    }

    @Override
    public void simulationInit() {
        elevator.simulationInit();
    }

    @Override
    public void simulationPeriodic() {
        elevator.simulationPeriodic();
    }

    @Override
    public void updateShuffleboard() {
        elevator.updateShuffleboard();
    }

    @Override
    public void configShuffleboard() {
        elevator.configShuffleboard();
    }

    // TODO IMPLEMENT
    @Override
    public boolean checkInitStatus() {
        return super.checkInitStatus() && elevator.checkInitStatus();
    }

    public Command goToSetpoint(double setpoint) {
        return new WaitCommand(0);
    }
}
