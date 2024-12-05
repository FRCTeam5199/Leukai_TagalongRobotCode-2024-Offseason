// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;


import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.AmpTrapCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.AmpTrap;

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

        public static final Command rightBumperCommand =
        new SelectCommand<>(
            // Maps selector values to commands
            Map.ofEntries(
                Map.entry(Mode.SHOOTER, new PrintCommand("shoot shooter")),
                Map.entry(Mode.AMP, new PrintCommand("shoot amp")),
                Map.entry(Mode.CLIMB, new PrintCommand("shoot amp")),
                Map.entry(Mode.SHUTTLE, new PrintCommand("shoot shooter"))
            ),
                RobotContainer::select
        );

        public static final Command leftBumperCommand =
        new SelectCommand<>(
            // Maps selector values to commands
            Map.ofEntries(
                Map.entry(Mode.SHOOTER, new ParallelCommandGroup(
                    ShooterCommands.aimShooterMid()
                        .andThen(ShooterCommands.blankShoot()),
                    new PrintCommand("Mid shot aim and rev"))),
                Map.entry(Mode.AMP, new ParallelCommandGroup(
                    new PrintCommand("do nothing"))),
                Map.entry(Mode.CLIMB, new ParallelCommandGroup(
                    new PrintCommand("elevator aim"),
                    AmpTrapCommands.aimElevatorTrap()
                    )
                ),
                Map.entry(Mode.SHUTTLE, new ParallelCommandGroup(
                    new PrintCommand("low shooter"),
                    ShooterCommands.stablizeShooter()
                ))
            ),
                RobotContainer::select
        );
    public static final Command leftBumperFalseCommand =
        new SelectCommand<>(
            // Maps selector values to commands
            Map.ofEntries(
                Map.entry(Mode.SHOOTER, new ParallelCommandGroup(
                    ShooterCommands.stablizeShooter(),
                    new PrintCommand("Mid shot aim and rev"))),
                Map.entry(Mode.AMP, new ParallelCommandGroup(
                    new PrintCommand("do nothing"))),
                Map.entry(Mode.CLIMB, new ParallelCommandGroup(
                    new PrintCommand("elevator aim"),
                    AmpTrapCommands.aimElevatorStable()
                    )
                ),
                Map.entry(Mode.SHUTTLE, new ParallelCommandGroup(
                    new PrintCommand("low shooter"),
                    ShooterCommands.stablizeShooter()
                ))
            ),
                RobotContainer::select
        );
}
