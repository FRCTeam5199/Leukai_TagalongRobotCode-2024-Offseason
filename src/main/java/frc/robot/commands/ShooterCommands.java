// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.ShooterSubsystem;
/** Add your docs here. */
public class ShooterCommands {
    ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    public static Command blankShoot() {
        return new FunctionalCommand(
            //TODO: tune?
            ()->{},
            ()->ShooterSubsystem.getInstance().setShooterSpeeds(20), 
            (something)->{},
            ()->false,
            ShooterSubsystem.getInstance()
        );
    }
    public Command stopShooter() {
        return shooterSubsystem.runOnce(() -> {
            shooterSubsystem.setShooterSpeeds(0);
        });
    }
    public Command aimShooterMax() {
        return shooterSubsystem.runOnce(() -> {
            shooterSubsystem.moveShooterToSetpoint(ShooterPivotAngles.MAX.getDegrees());
        });
    }
}
