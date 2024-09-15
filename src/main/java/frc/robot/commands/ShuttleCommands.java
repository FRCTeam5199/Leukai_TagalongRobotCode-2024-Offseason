package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;

public class ShuttleCommands {
    private static final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private static final CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.DriveTrain;


}
