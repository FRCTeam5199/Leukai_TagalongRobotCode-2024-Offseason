package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ObjectDetectionSubsystem;

public class Autos extends Command {
    private static Autos autos;

    public boolean part1Finished;
    public boolean part2Finished = false;
    public boolean alt1 = false;
    
    public static ObjectDetectionSubsystem objectDetection = ObjectDetectionSubsystem.getInstance();
    SwerveRequest.ApplyChassisSpeeds autonDrive = new SwerveRequest.ApplyChassisSpeeds();
    HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
            new com.pathplanner.lib.util.PIDConstants(5, 0, 0),
            new com.pathplanner.lib.util.PIDConstants(5, 0, 0),
            5.76072, .375, new ReplanningConfig(true, true));
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
        return AutoBuilder.buildAuto("6 piece red shoot");
    }

    public Command sixPieceRedPart1(){
        return AutoBuilder.buildAuto("6 piece red part 1");
    }

    public Command sixPieceRedPart2(){
        part2Finished = false;
        return AutoBuilder.buildAuto("6 piece red part 2").andThen(()-> part2Finished = true);
    }

    public Command sixPieceRedNote5Check(){
        return AutoBuilder.buildAuto("6 piece red note 5 check");
    }

    public Command sixPieceRedPart2Alt1(){
        alt1 = false;
        return AutoBuilder.buildAuto("6 piece red part 2 alt 1").andThen(()-> alt1 = true);
    }
        public Command sixPieceRedPart2Alt2(){
        return AutoBuilder.buildAuto("6 piece red part 2 alt 2");
    }

    public Command sixPiece(){
        return sixPieceRedPart1();
    }

   
    public Command sixPieceRedwithAlt() {
        return new SequentialCommandGroup(
            sixPieceRedPart1(),
            sixPieceRedPart2().unless(()-> !objectDetection.notePresent()),
            sixPieceRedNote5Check().unless(()-> part2Finished == true),
            sixPieceRedPart2Alt1().unless(()-> !objectDetection.notePresent()),
            sixPieceRedPart2Alt2().unless(()-> alt1 == true || part2Finished == true)
        );
    }

    public Command fourPieceRedMiddle(){
        return AutoBuilder.buildAuto("4 piece source middle");
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