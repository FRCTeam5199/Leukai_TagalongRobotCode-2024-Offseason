// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.commands.base.PivotToCommand;
import frc.robot.subsystems.ShooterSubsystem;
/** Add your docs here. */
public class ShooterCommands {
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
    public static Command stopShooter() {
        return new FunctionalCommand(
                //TODO: tune?
                ()->{},
                ()->ShooterSubsystem.getInstance().setShooterSpeeds(0),
                (something)->{},
                ()->true,
                ShooterSubsystem.getInstance()
        );
    }
    public static Command unAimShooter() {
        return new PivotToCommand<>(ShooterSubsystem.getInstance(), ShooterPivotAngles.LOW.getRotations(), false);
    }
    public static Command aimShooterMid(boolean holdPos) {
        return new PivotToCommand<>(ShooterSubsystem.getInstance(), ShooterPivotAngles.MID.getRotations(), holdPos);
    }
}