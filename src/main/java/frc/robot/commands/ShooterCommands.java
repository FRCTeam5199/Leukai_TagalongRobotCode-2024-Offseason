// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.commands.base.PivotToCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
/** Add your docs here. */
public class ShooterCommands {
    /*
     * Shoot 
     */
    static ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    static IndexerSubsystem indexersubsystem = IndexerSubsystem.getInstance();
    public static Command blankShoot() {
        return new FunctionalCommand(
            //TODO: don't tune?
            ()->shooterSubsystem.setShooterSpeeds(60),
            ()->{}, 
            (something)->shooterSubsystem.setShooterSpeeds(0),
            ()->false,
            shooterSubsystem
        );
    }
    public static Command stablizeShooter() {
        return new PivotToCommand<>(shooterSubsystem, ShooterPivotAngles.STABLE.getRotations(), true);
    }
    public static Command aimShooterLowShuttle() {
        return new PivotToCommand<>(shooterSubsystem, ShooterPivotAngles.LOW_SHUTTlE.getRotations(), true);
    }
    public static Command aimShooterLow() {
        return new PivotToCommand<>(shooterSubsystem, ShooterPivotAngles.LOW.getRotations(), true);
    }
    public static Command aimShooterMid() {
        return new PivotToCommand<>(shooterSubsystem, ShooterPivotAngles.MID.getRotations(), true);
    }
    public static Command aimShooterHighShuttle() {
        return new PivotToCommand<>(shooterSubsystem, ShooterPivotAngles.HIGH_SHUTTLE.getRotations(), true);
    }
    //50, -75, 50
    public static Command moveIntoShooter() {
        return new FunctionalCommand(
            () -> indexersubsystem.setRollerSpeeds(50, -75, 50),
            () -> {},
            (interrupted) -> indexersubsystem.setRollerSpeeds(0, 0, 0),
            () -> false,
            indexersubsystem
            );
    }

    public static Command aimShooterClimb() {
        return new PivotToCommand<>(shooterSubsystem, ShooterPivotAngles.MAX.getRotations(), true);
    }
}