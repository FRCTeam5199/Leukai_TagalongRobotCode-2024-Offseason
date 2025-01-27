package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.commands.base.ClimberHeights;
import frc.robot.parsers.ClimberParser;
import frc.robot.parsers.json.ClimberConfJson;
import frc.robot.subsystems.minor.TagalongElevator;
import frc.robot.tagalong.ElevatorAugment;
import frc.robot.tagalong.TagalongSubsystemBase;

public class Climber extends TagalongSubsystemBase implements ElevatorAugment {
    private static Climber climberSubsystem;
    public final ClimberParser _climberParser;
    public final ClimberConfJson _climberConfRight;
    public final ClimberConfJson _climberConfLeft;
    private final TagalongElevator _elevatorRight;
    private final TagalongElevator _elevatorLeft;

    public Climber(String filePath) {
        this(filePath == null ? null : new ClimberParser(Filesystem.getDeployDirectory(), filePath));
    }

    public Climber(ClimberParser parser) {
        super(parser);
        _climberParser = parser;

        if (_configuredDisable) {
            // _io = null;
            _climberConfRight = null;
            _climberConfLeft = null;
            _elevatorRight = new TagalongElevator(null);
            _elevatorLeft = new TagalongElevator(null);
            return;
        }

        _climberConfRight = _climberParser.climberConf;
        _climberConfLeft = _climberParser.climberConf;
        _elevatorRight = new TagalongElevator(parser.elevatorRightParser);
        _elevatorLeft = new TagalongElevator(parser.elevatorLeftParser);

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

    /* -------- Logging: utilities and configs -------- */
    // private final ClimberIOTalonFX _io;
    // private final ClimberIOInputsAutoLogged _inputs = new ClimberIOInputsAutoLogged();

    public static Climber getInstance() {
        if (climberSubsystem == null) {
            climberSubsystem = new Climber("configs/climber/climberConf.json");
        }
        return climberSubsystem;
    }

    @Override
    public TagalongElevator getElevator() {
        return _elevatorRight;
    }

    @Override
    public TagalongElevator getElevator(int i) {
        switch (i) {
            case 1:
                return _elevatorLeft;
            default:
                return _elevatorRight;
        }
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
        _elevatorRight.onEnable();
        _elevatorLeft.onEnable();
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

    public void setElevatorProfiles(ClimberHeights leftClimberHeight, ClimberHeights rightClimberHeights) {
        _elevatorLeft.setElevatorProfile(leftClimberHeight.getHeightM(), 0);
        _elevatorRight.setElevatorProfile(rightClimberHeights.getHeightM(), 0);
    }

    public void followLastElevatorProfiles() {
        _elevatorLeft.followLastProfile();
        _elevatorRight.followLastProfile();
    }

    public void setHoldElevatorPositions() {
        _elevatorLeft.setHoldElevatorPosition(true);
        _elevatorRight.setHoldElevatorPosition(true);
    }

    public boolean isElevatorProfilesFinished() {
        return _elevatorLeft.isProfileFinished() && _elevatorRight.isProfileFinished();
    }

    public void setClimberPowers(double percent) {
        _elevatorLeft.setElevatorPower(percent);
        _elevatorRight.setElevatorPower(percent);
    }

    public static final class ElevatorConstants {
        public static final int ELEVATOR_RIGHT_ID = 0;
        public static final int ELEVATOR_LEFT_ID = 1;

        public static final double ELEVATOR_ZEROING_SPEED_MPS = -0.06;
        public static final double ELEVATOR_PREP_SPEED_MPS = -0.01;
        public static final double ELEVATOR_ZEROING_STALL_TOLERANCE = 50;
        public static final int ELEVATOR_ZEROING_STALL_LOOPS = 6;
    }
}
