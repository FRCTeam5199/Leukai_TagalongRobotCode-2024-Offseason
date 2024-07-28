package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.base.ElevatorRaiseToCommand;
import frc.robot.commands.base.PivotToCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.NoteElevator;
import frc.robot.subsystems.Shooter;
import frc.robot.utility.LookUpTable;

public class ScoreCommands {
    private static final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
    private static final NoteElevator elevatorSubsystem = NoteElevator.getInstance();
    private static final Shooter shooterSubsystem = Shooter.getInstance();
    private static TrapezoidProfile driveRotationalTrapezoidProfile;

    public static Command driveAutoTurn(CommandXboxController commandXboxController,
            CommandSwerveDrivetrain commandSwerveDrivetrain, FieldCentric fieldCentricSwerveDrive) {
        return new ConditionalCommand(
            turnDriveToSpeaker(commandXboxController, commandSwerveDrivetrain, fieldCentricSwerveDrive, 0.0, 16.58, 5.59),
            turnDriveToSpeaker(commandXboxController, commandSwerveDrivetrain, fieldCentricSwerveDrive, 180.0, -0.0381, 5.48),
                () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red);
    }

    private static Command turnDriveToSpeaker(CommandXboxController commandXboxController, CommandSwerveDrivetrain commandSwerveDrivetrain, FieldCentric fieldCentricSwerveDrive, double offset, double locX, double locY) {
        return new FunctionalCommand(null,
                        () -> commandSwerveDrivetrain.applyRequest(
                                () -> fieldCentricSwerveDrive.withVelocityX(-commandXboxController.getLeftY())
                                        .withVelocityY(-commandXboxController.getLeftX())
                                        .withRotationalRate(driveRotationalTrapezoidProfile.calculate(1.0,
                                                (new State(
                                                        commandSwerveDrivetrain.getPose().getRotation()
                                                                .plus(Rotation2d.fromDegrees(offset)).getDegrees(),
                                                        commandSwerveDrivetrain.getPigeon2().getRate())),
                                                new State(Units.radiansToDegrees(Math.atan(
                                                        (locY - commandSwerveDrivetrain.getPose().getY()) / (locX
                                                                - commandSwerveDrivetrain.getPose().getX()))),
                                                        2.27)).velocity)),
                        null,
                        null);
    }

    public static Command elevatorStable() {
        return new ParallelCommandGroup(
                new ElevatorRaiseToCommand<>(elevatorSubsystem, () -> 0),
                new InstantCommand(() -> indexerSubsystem.setRollerSpeeds(0, 0, 0)));
    }

    public static Command ampScore() {
        return new SequentialCommandGroup(
                new ElevatorRaiseToCommand<>(elevatorSubsystem, () -> 30),
                new FunctionalCommand(
                        () -> indexerSubsystem.setRollerSpeeds(0, 0.4, 0),
                        () -> {
                        },
                        interrupted -> elevatorStable(),
                        () -> false,
                        indexerSubsystem).until(() -> !indexerSubsystem.isNoteInAmpTrap()))
                .unless(() -> !indexerSubsystem.isNoteInAmpTrap());
    }

    public static Command basicAutoShootCommand() {
        return new SequentialCommandGroup(
                new PivotToCommand<>(shooterSubsystem.getPivot(), 15.0, true),
                new WaitUntilCommand(shooterSubsystem::reachedShootingConditions),
                indexerFeedCommand(indexerSubsystem).until(shooterSubsystem::shotNote))
                .deadlineWith(flywheelSpinupCommand());
    }

    public static Command flywheelSpinupCommand() {
        return new FunctionalCommand(
                () -> {
                    shooterSubsystem.getFlywheel(0).setFlywheelControl(3000, true);
                    shooterSubsystem.getFlywheel(1).setFlywheelControl(4000, true);
                },
                () -> {
                },
                interrupted -> {
                    shooterSubsystem.getFlywheel(0).setFlywheelPower(0);
                    shooterSubsystem.getFlywheel(1).setFlywheelPower(0);
                },
                () -> false,
                shooterSubsystem);
    }

    public static Command indexerFeedCommand(IndexerSubsystem indexerSubsystem) {
        return new FunctionalCommand(
                () -> indexerSubsystem.getRoller().setRollerPower(0.5),
                () -> {
                },
                interrupted -> indexerSubsystem.getRoller().setRollerPower(0),
                () -> false,
                indexerSubsystem);
    }

    public static Command autoAim() {
        return new FunctionalCommand(
                () -> {
                },
                () -> {
                    new PivotToCommand<>(shooterSubsystem.getPivot(), LookUpTable.findValue(0).armAngle, true);
                    shooterSubsystem.getFlywheel(0).setFlywheelPower(LookUpTable.findValue(0).shooterRPM);
                    shooterSubsystem.getFlywheel(1).setFlywheelPower(LookUpTable.findValue(0).shooterRPM);
                },
                interrupted -> {
                    new PivotToCommand<>(shooterSubsystem.getPivot(), 0, true);
                    shooterSubsystem.getFlywheel(0).setFlywheelPower(0);
                    shooterSubsystem.getFlywheel(1).setFlywheelPower(0);
                },
                () -> !indexerSubsystem.isNoteInIndexer());
    }
}
