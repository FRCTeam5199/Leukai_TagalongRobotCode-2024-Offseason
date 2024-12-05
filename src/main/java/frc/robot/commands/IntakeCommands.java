// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IndexerSubsystem;
public class IntakeCommands{
  static IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();

  /**
   * intakes into amp
   * @return
   */
  public static Command IntakeToAmp()
  {
    return new FunctionalCommand
    (
     () -> {indexerSubsystem.setRollerSpeeds(100,60,0); },
     () -> {}, 
     (interrupted) -> indexerSubsystem.setRollerSpeeds(0,0,0), 
     () -> indexerSubsystem.isNoteInAmpTrap(),
     indexerSubsystem
    );
  }
   /**
    * Make sure to use this with .onfalse after EVERY intake command
    * @return
    */
    public static Command IndexerOff()
  {
    return new FunctionalCommand
    (
     () -> {indexerSubsystem.setRollerSpeeds(0,0,0); },
     () -> {}, 
     (interrupted) -> indexerSubsystem.setRollerSpeeds(0,0,0), 
     () -> false,
     indexerSubsystem
    );
  }
  
    public static Command IntakeToShooter()
  {
    return new FunctionalCommand
    (
     () -> {indexerSubsystem.setRollerSpeeds(100,-60,10); },
     () -> {}, 
     (interrupted) -> indexerSubsystem.setRollerSpeeds(0,0,0), 
     () -> indexerSubsystem.isNoteInIndexer(),
     indexerSubsystem
    );
  }

    public static Command AmpScore()
  {
    return new FunctionalCommand
    (
     () -> {indexerSubsystem.setRollerSpeeds(0,50,0); },
     () -> {}, 
     (interrupted) -> indexerSubsystem.setRollerSpeeds(0,0,0), 
     () -> false,
     indexerSubsystem
    );
  }

    public static Command TrapScore()
  {
    return new FunctionalCommand
    (
     () -> {indexerSubsystem.setRollerSpeeds(0,100,0); },
     () -> {}, 
     (interrupted) -> indexerSubsystem.setRollerSpeeds(0,0,0), 
     () -> false,
     indexerSubsystem
    );
  }

}
