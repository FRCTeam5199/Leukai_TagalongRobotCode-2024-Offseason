// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;


import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.AmpTrapCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.ShooterSubsystem;

/** Add your docs here. */
public class CommandSelections {

       public static final Command rightTriggerCommand =
        new SelectCommand<>(
            // Maps selector values to commands
            Map.ofEntries(
                Map.entry(Mode.SHOOTER, new PrintCommand("Intake into shooter")),
                Map.entry(Mode.AMP, new PrintCommand("Intake into amp")),
                Map.entry(Mode.CLIMB, new PrintCommand("Climb down")),
                Map.entry(Mode.SHUTTLE, new PrintCommand("Intake into shooter"))
            ),
                RobotContainer::select
        );

        public static final Command leftTriggerCommand =
        new SelectCommand<>(
            // Maps selector values to commands
            Map.ofEntries(
                Map.entry(Mode.SHOOTER, new PrintCommand("Sub rev and aim")),
                Map.entry(Mode.AMP, new PrintCommand("Elevator aim")),
                Map.entry(Mode.CLIMB, new PrintCommand("Climb up")),
                Map.entry(Mode.SHUTTLE, new PrintCommand("Hi shuttle setpoint and rev"))
            ),
                RobotContainer::select
        );

        public static final Command rightBumperFalseCommand =
        new SelectCommand<>(
            // Maps selector values to commands
            Map.ofEntries(
                Map.entry(Mode.SHOOTER, new PrintCommand("shoot shooter").andThen(ShooterCommands.stablizeShooter())),
                Map.entry(Mode.AMP, new PrintCommand("shoot amp").andThen(AmpTrapCommands.resetElevatorAndIndex())),
                Map.entry(Mode.CLIMB, new PrintCommand("shoot amp").andThen(AmpTrapCommands.resetElevatorAndIndex())),
                Map.entry(Mode.SHUTTLE, new PrintCommand("shoot shooter").andThen(ShooterCommands.stablizeShooter()))
            ),
                RobotContainer::select
        );

    public static final Command rightBumperCommand =
            new SelectCommand<>(
                    // Maps selector values to commands
                    Map.ofEntries(
                            Map.entry(Mode.SHOOTER, new PrintCommand("shoot shooter").andThen(ShooterCommands.blankShoot())),
                            Map.entry(Mode.AMP, new PrintCommand("shoot amp").andThen(AmpTrapCommands.shootAmp())),
                            Map.entry(Mode.CLIMB, new PrintCommand("shoot amp").andThen(AmpTrapCommands.shootTrap())),
                            Map.entry(Mode.SHUTTLE, new PrintCommand("shoot shooter").andThen(ShooterCommands.blankShoot()))
                    ),
                    RobotContainer::select
            );

        public static final Command leftBumperCommand =
        new SelectCommand<>(
            // Maps selector values to commands
            Map.ofEntries(
                Map.entry(Mode.SHOOTER, new PrintCommand("Mid shot rev and aim")),
                Map.entry(Mode.AMP, new PrintCommand("do nothing")),
                Map.entry(Mode.CLIMB, new PrintCommand("elevator aim")),
                Map.entry(Mode.SHUTTLE, new PrintCommand("low shooter"))
            ),
                RobotContainer::select
        );

}
