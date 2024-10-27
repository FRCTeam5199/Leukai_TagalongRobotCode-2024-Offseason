package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.commands.base.PivotToCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ObjectDetectionSubsystem;

public class Autos extends Command {

    public static SendableChooser<Command> autonChooserRed = new SendableChooser<>();
    public static SendableChooser<Command> autonChooserBlue = new SendableChooser<>();
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
    private SendableChooser<Command> autoChooser = new SendableChooser<Command>();
    private Command twoPieceExtendedBlue;

    private Command twoPieceExtendedRed;
    private Command twoAndAHalfPieceExtendedRed;
    private Command twoAndAHalfPieceExtendedBlue;
    private Command threePieceExtendedRed;
    private Command threePieceExtendedBlue;
    private Command fourPieceRed;
    private Command fourPieceBlue;
    private Command fivePieceAmpRed;
    private Command fivePieceAmpBlue;

    public Autos(CommandSwerveDrivetrain swerveDrive) {
        this.swerveDrive = swerveDrive;
        AutoBuilder.configureHolonomic(() -> swerveDrive.getPose(),
                swerveDrive::seedFieldRelative, swerveDrive::getCurrentRobotChassisSpeeds,
                (speeds) -> swerveDrive.setControl(autonDrive.withSpeeds(speeds)),
                pathFollowerConfig, () -> false, swerveDrive);

        NamedCommands.registerCommand("intake", IntakeCommands.intake());

        NamedCommands.registerCommand("subWooferShotNoMove", new InstantCommand(() -> aiming.initialize()));
        NamedCommands.registerCommand("updateShotSub",
                new InstantCommand(() -> aiming.updateSetpoint(ShooterPivotAngles.MAX.getDegrees())));
        NamedCommands.registerCommand("autoShootSub",
                ScoreCommands.setShooterSpeeds(60)
                        .until(() -> RobotContainer.shooterSubsystem.reachedShootingCondtions(45))
                        .andThen(ScoreCommands.indexerFeedCommandAutoStop(45))
                        .until(() -> !RobotContainer.indexerSubsystem.isNoteInIndexer())
        );
        NamedCommands.registerCommand("autoShootSubFast",
                ScoreCommands.setShooterSpeeds(70)
                        .until(() -> RobotContainer.shooterSubsystem.reachedShootingCondtions(45))
                        .andThen(ScoreCommands.indexerFeedCommandAutoStop(45))
                        .until(() -> !RobotContainer.indexerSubsystem.isNoteInIndexer())
        );

        NamedCommands.registerCommand("driveAutoAim", ScoreCommands.autonAutoTurn(new SwerveRequest.FieldCentric())
                .andThen(new WaitCommand(.5)));
        NamedCommands.registerCommand("driveAutoAimFast", ScoreCommands.autonAutoTurn(new SwerveRequest.FieldCentric()));

        NamedCommands.registerCommand("updateShot",
                new InstantCommand(() -> aiming.updateSetpoint(RobotContainer.armAutoAimAngle)));

        NamedCommands.registerCommand("updateShot5PieceFarRed",
                new InstantCommand(() -> RobotContainer.shooterSubsystem.setShooterSpeeds(70))
                        .andThen(() -> aiming.updateSetpoint(23.25)));
        NamedCommands.registerCommand("updateShot5Piece1Red",
                new InstantCommand(() -> aiming.updateSetpoint(43)));
        NamedCommands.registerCommand("updateShot5Piece2Red",
                new InstantCommand(() -> aiming.updateSetpoint(29.75)));
        NamedCommands.registerCommand("updateShot5Piece3Red",
                new InstantCommand(() -> aiming.updateSetpoint(38)));

        NamedCommands.registerCommand("autoShootWithCheck",
                new InstantCommand(() -> RobotContainer.shooterSubsystem.setShooterSpeeds(60))
                        .andThen(new WaitCommand(0.2))
                        .andThen(() -> aiming.updateSetpoint(RobotContainer.armAutoAimAngle))
                        .andThen(ScoreCommands.indexerFeedCommandAutoStop(60))
        );
        NamedCommands.registerCommand("autoShootWithCheckFast",
                new InstantCommand(() -> RobotContainer.shooterSubsystem.setShooterSpeeds(70))
                        .andThen(new WaitCommand(0.2))
                        .andThen(() -> aiming.updateSetpoint(RobotContainer.armAutoAimAngle))
                        .andThen(ScoreCommands.indexerFeedCommandAutoStop(70))
        );
        NamedCommands.registerCommand("autoShootWithCheckSetpoint",
                new InstantCommand(() -> RobotContainer.shooterSubsystem.setShooterSpeeds(60))
                        .andThen(ScoreCommands.indexerFeedCommandAutoStop(60))
        );
        NamedCommands.registerCommand("autoShootWithCheckSetpointFast",
                new InstantCommand(() -> RobotContainer.shooterSubsystem.setShooterSpeeds(70))
                        .andThen(ScoreCommands.indexerFeedCommandAutoStop(70))
        );

        twoPieceExtendedRed = twoPieceRedExtended();
        twoPieceExtendedBlue = twoPieceBlueExtended();
        twoAndAHalfPieceExtendedRed = twoAndAHalfPieceRedExtended();
        twoAndAHalfPieceExtendedBlue = twoAndAHalfPieceBlueExtended();
        threePieceExtendedRed = threePieceRedExtended();
        threePieceExtendedBlue = threePieceBlueExtended();
        fourPieceRed = fourPieceBRed();
        fourPieceBlue = fourPieceBBlue();
        fivePieceAmpRed = fivePieceAmpRed();
        fivePieceAmpBlue = fivePieceBlue();

        Shuffleboard.getTab("Autons").add("Red Autons", autonChooserRed).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(2, 1);
        Shuffleboard.getTab("Autons").add("Blue Autons", autonChooserBlue).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(2, 1);

        autonChooserRed.addOption("Shoot Do Nothing", shootNoMove());
        autonChooserRed.addOption("2 Piece", twoPieceExtendedRed);
        autonChooserRed.addOption(" 2.5 Piece Extended", twoAndAHalfPieceExtendedRed);
        autonChooserRed.addOption(" 3 Piece Extended", threePieceExtendedRed);
        autonChooserRed.setDefaultOption("4 Piece", fourPieceRed);
        autonChooserRed.addOption("5 Piece Amp", fivePieceAmpRed);

        autonChooserBlue.addOption("Shoot Do Nothing", shootNoMove());
        autonChooserBlue.addOption("2 Piece", twoPieceExtendedBlue);
        autonChooserBlue.addOption("2.5 Piece Extended", twoAndAHalfPieceExtendedBlue);
        autonChooserBlue.addOption("3 Piece Extended", threePieceExtendedBlue);
        autonChooserBlue.setDefaultOption("4 Piece", fourPieceBlue);
        autonChooserBlue.addOption("5 Piece Amp", fivePieceAmpBlue);
    }

    public static Autos getInstance(CommandSwerveDrivetrain commandSwerveDriveTrain) {
        if (autos == null) autos = new Autos(commandSwerveDriveTrain);
        return autos;
    }

    public Command sixPieceRed() {
        return AutoBuilder.buildAuto("6 piece red shoot");
    }

    public Command fourPieceBlue() {
        return AutoBuilder.buildAuto("4 piece blue");
    }

    public Command fourPieceRed() {
        return AutoBuilder.buildAuto("4 piece red");
    }

    public Command fivePieceRed() {
        return AutoBuilder.buildAuto("5 piece red shoot");
    }

    public Command twoPieceBlueExtended() {
        return AutoBuilder.buildAuto("2 piece blue");
    }

    public Command fivePieceAmpRed() {
        return AutoBuilder.buildAuto("5 piece amp red");
    }

    public Command fivePieceAmpBlue() {
        return AutoBuilder.buildAuto("5 piece amp blue");
    }

    public Command fivePieceMiddleRed() {
        return AutoBuilder.buildAuto("5 piece middle red");
    }

    public Command fivePieceBlue() {
        return AutoBuilder.buildAuto("5 piece blue");
    }

    public Command sixPieceRedPart1() {
        return AutoBuilder.buildAuto("6 piece red part 1");
    }

    public Command sixPieceRedPart2() {
        part2Finished = false;
        return AutoBuilder.buildAuto("6 piece red part 2").andThen(() -> part2Finished = true);
    }

    public Command twoPieceRedExtended() {
        return AutoBuilder.buildAuto("2 piece red");
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


    public Command twoAndAHalfPieceRedExtended() {
        return AutoBuilder.buildAuto("2.5 piece extended red");
    }

    public Command twoAndAHalfPieceBlueExtended() {
        return AutoBuilder.buildAuto("2.5 piece extended blue");
    }

    public Command threePieceRedExtended() {
        return AutoBuilder.buildAuto("3 Piece Extended Red");

    }

    public Command threePieceBlueExtended() {
        return AutoBuilder.buildAuto("3 piece extended blue");
    }

    public Command shootNoMove() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> aiming.initialize()),
                new InstantCommand(() -> aiming.updateSetpoint(RobotContainer.armAutoAimAngle)),
                ScoreCommands.setShooterSpeeds(60)
                        .until(() -> RobotContainer.shooterSubsystem.reachedShootingCondtions(50)),
                ScoreCommands.indexerFeedCommandAutoStop(50)
                        .until(() -> !RobotContainer.indexerSubsystem.isNoteInIndexer())
        );
    }

    public Command fourPieceBRed() {
        return AutoBuilder.buildAuto("4 piece B red");
    }

    public Command fourPieceBBlue() {
        return AutoBuilder.buildAuto("blue");
    }
}