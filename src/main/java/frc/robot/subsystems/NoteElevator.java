package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.parsers.NotevatorParser;
import frc.robot.parsers.json.NotevatorConfJson;
import frc.robot.subsystems.minor.TagalongDualMotorElevator;
import frc.robot.subsystems.minor.TagalongElevator;
import frc.robot.tagalong.ElevatorAugment;
import frc.robot.tagalong.TagalongSubsystemBase;

public class NoteElevator extends TagalongSubsystemBase implements ElevatorAugment {
    private static NoteElevator elevatorSubsystem;
    public final NotevatorParser _elevatorParser;
    public final NotevatorConfJson _notevatorConf;
    private final TagalongDualMotorElevator _elevator;

    public NoteElevator(String filePath) {
        this(filePath == null ? null : new NotevatorParser(Filesystem.getDeployDirectory(), filePath));
    }

    public NoteElevator(NotevatorParser parser) {
        super(parser);
        _elevatorParser = parser;

        if (_configuredDisable) {
            // _io = null;
            _notevatorConf = null;
            _elevator = new TagalongDualMotorElevator(null);
            return;
        }

        _notevatorConf = _elevatorParser.notevatorConf;
        _elevator = new TagalongDualMotorElevator(_elevatorParser.elevatorParser);

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

    public static NoteElevator getInstance() {
        if (elevatorSubsystem == null) {
            elevatorSubsystem = new NoteElevator("configs/notevator/notevatorConf.json");
        }
        return elevatorSubsystem;
    }

    /* -------- Logging: utilities and configs -------- */
    // private final ClimberIOTalonFX _io;
    // private final ClimberIOInputsAutoLogged _inputs = new ClimberIOInputsAutoLogged();

    @Override
    public TagalongElevator getElevator() {
        return _elevator;
    }

    @Override
    public TagalongElevator getElevator(int i) {
        return _elevator;
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
        _elevator.onEnable();
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
        _elevator.onDisable();
    }

    @Override
    public void periodic() {
        if (_isSubsystemDisabled) {
            return;
        }
        _elevator.periodic();

        // Logging
        // _io.updateInputs(_inputs);
        // Logger.processInputs("Climber", _inputs);
        updateShuffleboard();
    }

    @Override
    public void disabledPeriodic() {
        _elevator.disabledPeriodic();
    }

    @Override
    public void simulationInit() {
        _elevator.simulationInit();
    }

    @Override
    public void simulationPeriodic() {
        _elevator.simulationPeriodic();
    }

    @Override
    public void updateShuffleboard() {
        _elevator.updateShuffleboard();
    }

    @Override
    public void configShuffleboard() {
        _elevator.configShuffleboard();
    }

    // TODO IMPLEMENT
    @Override
    public boolean checkInitStatus() {
        return super.checkInitStatus() && _elevator.checkInitStatus();
    }

    public Command goToSetpoint(double setpoint) {
        return new WaitCommand(0);
    }

    public static final class ElevatorConstants {
        public static final int ELEVATOR_ID = 0;

        public static final double ELEVATOR_ZEROING_SPEED_MPS = -0.06;
        public static final double ELEVATOR_PREP_SPEED_MPS = -0.01;
        public static final double ELEVATOR_ZEROING_STALL_TOLERANCE = 50;
        public static final int ELEVATOR_ZEROING_STALL_LOOPS = 6;

        public static final double ELEVATOR_AMP_MPS = 1.0;
        public static final double ELEVATOR_TRAP_MPS = 1.0;
    }

    public static final class RollerConstants {
        public static final double ROLLER_AMP_SHOT = 50.0;
    }
}
