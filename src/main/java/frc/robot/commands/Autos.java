package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.commands.base.PivotToCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ObjectDetectionSubsystem;

public class Autos extends Command {
    public static PivotToCommand aimingWhileMoving = new PivotToCommand<>(RobotContainer.shooterSubsystem, ShooterPivotAngles.STABLE.getRotations(), true);
    public static PivotToCommand aiming = new PivotToCommand<>(RobotContainer.shooterSubsystem, ShooterPivotAngles.STABLE.getRotations(), true);
    public static ObjectDetectionSubsystem objectDetection = ObjectDetectionSubsystem.getInstance();

    public boolean firstShot = false;
    private static Autos autos;
    public boolean part1Finished;
    public boolean part2Finished = false;
    public boolean alt1 = false;
    SwerveRequest.ApplyChassisSpeeds autonDrive = new SwerveRequest.ApplyChassisSpeeds();
    HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
            new com.pathplanner.lib.util.PIDConstants(5, 0, 0),
            new com.pathplanner.lib.util.PIDConstants(4.5, 0, 0),
            6.5, .375, new ReplanningConfig(true, true));
    private CommandSwerveDrivetrain swerveDrive;
    private Map<String, Command> commandsMap = new HashMap<>();
    public SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    public Autos(CommandSwerveDrivetrain swerveDrive) {
        this.swerveDrive = swerveDrive;
        AutoBuilder.configureHolonomic(() -> swerveDrive.getPose(),
                swerveDrive::seedFieldRelative, swerveDrive::getCurrentRobotChassisSpeeds,
                (speeds) -> swerveDrive.setControl(autonDrive.withSpeeds(speeds)),
                pathFollowerConfig, () -> false, swerveDrive);

        NamedCommands.registerCommand("intake", IntakeCommands.intake());
        NamedCommands.registerCommand("autoShoot",
                ScoreCommands.setShooterSpeeds(60)
                        .until(() -> RobotContainer.shooterSubsystem.reachedShootingCondtions(60))
                        .andThen(ScoreCommands.indexerFeedCommandAutoStop(60))
                        .until(() -> !RobotContainer.indexerSubsystem.isNoteInIndexer())
        );

        NamedCommands.registerCommand("subWooferShot", new InstantCommand(() -> aimingWhileMoving = new PivotToCommand(RobotContainer.shooterSubsystem, ShooterPivotAngles.MAX.getRotations(), true)));
        NamedCommands.registerCommand("autoShootSub",
                ScoreCommands.setShooterSpeeds(60)
                        .until(() -> RobotContainer.shooterSubsystem.reachedShootingCondtions(50))
                        .andThen(ScoreCommands.indexerFeedCommandAutoStop(50))
                        .until(() -> !RobotContainer.indexerSubsystem.isNoteInIndexer())
        );

        NamedCommands.registerCommand("autoShootFast",
                ScoreCommands.setShooterSpeeds(70)
                        .until(() -> RobotContainer.shooterSubsystem.reachedShootingCondtions(70))
                        .andThen(ScoreCommands.indexerFeedCommand(70))
                        .until(() -> !RobotContainer.indexerSubsystem.isNoteInIndexer())
        );

        NamedCommands.registerCommand("adjustPivotSpeed", new InstantCommand(() ->
                aimingWhileMoving = new PivotToCommand(RobotContainer.shooterSubsystem,
                        ShooterPivotAngles.STABLE.getRotations(), true, .01)));

        NamedCommands.registerCommand("driveAutoAim", ScoreCommands.autonAutoTurn(new SwerveRequest.FieldCentric()));

        NamedCommands.registerCommand("driveAndArmAim", ScoreCommands.autonAutoTurn(new SwerveRequest.FieldCentric())
                .alongWith(new InstantCommand()));
    }

    public static Autos getInstance(CommandSwerveDrivetrain commandSwerveDriveTrain) {
        if (autos == null) autos = new Autos(commandSwerveDriveTrain);
        return autos;
    }


//    private void initalizeCommandsMap() {
//        commandsMap.put("intakeCommand", IntakeCommands.intake());
//        commandsMap.put("prepShooterCommand", ScoreCommands.moveShooterToAutoAim(60));
//        commandsMap.put("autoShootCommand", AutonCommands.autonAutoShoot(60));
//        commandsMap.put("shootCommand", ScoreCommands.indexerFeedCommand(60));
//    }

    public Command sixPieceRed() {
        return AutoBuilder.buildAuto("6 piece red shoot");
    }

    public Command fourPieceBlue() {
        return AutoBuilder.buildAuto("4 piece blue");
    }

    public Command fourPieceRed() {
        return AutoBuilder.buildAuto("4 piece red");
    }

    public Command sixPieceRedPart1() {
        return AutoBuilder.buildAuto("6 piece red part 1");
    }

    public Command sixPieceRedPart2() {
        part2Finished = false;
        return AutoBuilder.buildAuto("6 piece red part 2").andThen(() -> part2Finished = true);
    }

    public Command sixPieceRedNote5Check() {
        return AutoBuilder.buildAuto("6 piece red note 5 check");
    }

    public Command sixPieceRedPart2Alt1() {
        alt1 = false;
        return AutoBuilder.buildAuto("6 piece red part 2 alt 1").andThen(() -> alt1 = true);
    }

    public Command sixPieceRedPart2Alt2() {
        return AutoBuilder.buildAuto("6 piece red part 2 alt 2");
    }

    public Command sixPiece() {
        return sixPieceRedPart1();
    }


    public Command sixPieceRedwithAlt() {
        return new SequentialCommandGroup(
                sixPieceRedPart1(),
                sixPieceRedPart2().unless(() -> !objectDetection.notePresent()),
                sixPieceRedNote5Check().unless(() -> part2Finished),
                sixPieceRedPart2Alt1().unless(() -> !objectDetection.notePresent()),
                sixPieceRedPart2Alt2().unless(() -> alt1 || part2Finished)
        );
    }

    public Command fourPieceRedMiddle() {
        return AutoBuilder.buildAuto("4 piece source middle");
    }


    public Command threePieceRedExtended() {
        return AutoBuilder.buildAuto("3 piece extended red");
    }

    public Command threePieceBlueExtended() {
        return AutoBuilder.buildAuto("3 piece extended blue");
    }
}