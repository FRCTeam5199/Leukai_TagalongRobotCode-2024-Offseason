// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ScoreCommands;
import frc.robot.commands.base.AutoShootCommands;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.NoteElevator;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    public static IndexerSubsystem indexerSubsystem = new IndexerSubsystem("configs/indexer/indexerConf.json");
    public static Shooter shooterSubsystem = new Shooter("configs/shooter/shooterConf.json");
    public static Climber climber = new Climber("configs/climber/climberConf.json");
    public static NoteElevator noteElevator = new NoteElevator("configs/notevator/notevatorConf.json");
    CommandXboxController commandXboxController = new CommandXboxController(Ports.DRIVER_XBOX_USB_PORT);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        commandXboxController.rightTrigger().onTrue(IntakeCommands.intake());
        commandXboxController.a().onTrue(IntakeCommands.switchAmpMode());
        commandXboxController.b().onTrue(IntakeCommands.switchShooterMode());

        commandXboxController.x().onTrue(ScoreCommands.ampScore());

        commandXboxController.leftTrigger().onTrue(ScoreCommands.basicAutoShootCommand());
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public void onEnable() {
        indexerSubsystem.onEnable();
        shooterSubsystem.onEnable();
    }

    public void onDisable() {
        indexerSubsystem.onDisable();
        shooterSubsystem.onDisable();
    }

    public void disabledPeriodic() {
        indexerSubsystem.disabledPeriodic();
        shooterSubsystem.disabledPeriodic();
    }

    public void simulationInit() {
        indexerSubsystem.simulationInit();
        shooterSubsystem.simulationInit();
    }

    public void simulationPeriodic() {
        indexerSubsystem.simulationPeriodic();
        shooterSubsystem.simulationPeriodic();
    }
}
