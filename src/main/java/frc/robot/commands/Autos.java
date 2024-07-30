package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Autos extends Command {
    private static Autos autos;
    SwerveRequest.ApplyChassisSpeeds autonDrive = new SwerveRequest.ApplyChassisSpeeds();
    HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
            new com.pathplanner.lib.util.PIDConstants(5, 0, 0),
            new com.pathplanner.lib.util.PIDConstants(5, 0, 0),
            5.76072, .375, new ReplanningConfig());
    private CommandSwerveDrivetrain swerveDrive;
    private Map<String, Command> commandsMap = new HashMap<>();
    private Map<String, Command> builtAutos = new HashMap<>();

    public Autos(CommandSwerveDrivetrain swerveDrive) {
        this.swerveDrive = swerveDrive;
        AutoBuilder.configureHolonomic(() -> swerveDrive.getPose(),
                swerveDrive::seedFieldRelative, swerveDrive::getCurrentRobotChassisSpeeds,
                (speeds) -> swerveDrive.setControl(autonDrive.withSpeeds(speeds)),
                pathFollowerConfig, () -> false, swerveDrive);

        NamedCommands.registerCommand("intake", IntakeCommands.intake());
        NamedCommands.registerCommand("autoShoot", ScoreCommands.moveShooterToAutoAimAndAutoShoot(60));
    }


//    public void init() {
//        initalizeCommandsMap();
//        NamedCommands.registerCommands(commandsMap);
//
//        buildAutos();
//    }

//    private void initalizeCommandsMap() {
//        commandsMap.put("intakeCommand", IntakeCommands.intake());
//        commandsMap.put("prepShooterCommand", ScoreCommands.moveShooterToAutoAim(60));
//        commandsMap.put("autoShootCommand", AutonCommands.autonAutoShoot(60));
//        commandsMap.put("shootCommand", ScoreCommands.indexerFeedCommand(60));
//    }

    private void buildAutos() {
        builtAutos.put("6 piece red", AutoBuilder.buildAuto("6 piece red"));
    }

    public Command getBuiltAuton(String builtAutonName) {
        return builtAutos.get(builtAutonName);
    }

    public Command sixPieceRed() {
        return AutoBuilder.buildAuto("6 piece red");
    }

    public Command testCommandsRed() {
        return AutoBuilder.buildAuto("Test commands red");
    }

    public Command testCommandsBlue() {
        return AutoBuilder.buildAuto("Test commands blue");
    }

    public Command testAuton() {
        return AutoBuilder.buildAuto("Test auton");
    }
}