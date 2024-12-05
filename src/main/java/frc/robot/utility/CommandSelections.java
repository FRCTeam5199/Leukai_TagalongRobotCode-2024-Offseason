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
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.AmpTrapCommands;
import frc.robot.commands.ClimberCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.AmpTrap;

/** Add your docs here. */
public class CommandSelections {

       public static final Command rightTriggerCommand =
        new SelectCommand<>(
            // Maps selector values to commands
            Map.ofEntries(
                Map.entry(Mode.SHOOTER, new PrintCommand("Intake into shooter").andThen(IntakeCommands.IntakeToShooter())),
                Map.entry(Mode.AMP, new PrintCommand("Intake into amp").andThen(IntakeCommands.IntakeToAmp())),
                Map.entry(Mode.CLIMB, new PrintCommand("Climb down").andThen(ClimberCommands.climbDown())),
                Map.entry(Mode.SHUTTLE, new PrintCommand("Intake into shooter").andThen(IntakeCommands.IntakeToShooter()))
            ),
                RobotContainer::select
        );

        public static final Command rightTriggerCommandFalse =
        new SelectCommand<>(
            // Maps selector values to commands
            Map.ofEntries(
                Map.entry(Mode.SHOOTER, new PrintCommand("Intake into shooter").andThen(IntakeCommands.IndexerOff())),
                Map.entry(Mode.AMP, new PrintCommand("Intake into amp").andThen(IntakeCommands.IndexerOff())),
                Map.entry(Mode.CLIMB, new PrintCommand("Climb down").andThen(ClimberCommands.climbStop())),
                Map.entry(Mode.SHUTTLE, new PrintCommand("Intake into shooter").andThen(IntakeCommands.IndexerOff()))
            ),
                RobotContainer::select
        );

        public static final Command leftTriggerCommand =
        new SelectCommand<>(
            // Maps selector values to commands
            Map.ofEntries(
                Map.entry(Mode.SHOOTER, new PrintCommand("Sub rev and aim").andThen(ShooterCommands.aimShooterSub()).andThen(ShooterCommands.blankShoot())),
                Map.entry(Mode.AMP, new PrintCommand("Elevator aim").andThen(AmpTrapCommands.aimElevatorAmp())),
                Map.entry(Mode.CLIMB, new PrintCommand("Climb up").andThen(ClimberCommands.climbUp())),
                Map.entry(Mode.SHUTTLE, new PrintCommand("Hi shuttle setpoint and rev").andThen(ShooterCommands.aimShooterHighShuttle().andThen(ShooterCommands.blankShoot())))
            ),
                RobotContainer::select
        );

        public static final Command leftTriggerCommandFalse =
        new SelectCommand<>(
            // Maps selector values to commands
            Map.ofEntries(
                Map.entry(Mode.SHOOTER, new PrintCommand("Sub rev and aim").andThen(ShooterCommands.stablizeShooter()).andThen(ShooterCommands.stopShoot())),
                Map.entry(Mode.AMP, new PrintCommand("Elevator aim").andThen(AmpTrapCommands.aimElevatorStable())),
                Map.entry(Mode.CLIMB, new PrintCommand("Climb up").andThen(ClimberCommands.climbStop())),
                Map.entry(Mode.SHUTTLE, new PrintCommand("Hi shuttle setpoint and rev").andThen(ShooterCommands.stablizeShooter().andThen(ShooterCommands.stopShoot())))
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
