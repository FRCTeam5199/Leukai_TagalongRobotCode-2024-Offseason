package frc.robot.commands.base;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.ScoreCommands;
import frc.robot.commands.ShooterPivotAngles;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.minor.TagalongPivot;
import frc.robot.tagalong.MathUtils;
import frc.robot.tagalong.PivotAugment;
import frc.robot.tagalong.TagalongSubsystemBase;

import java.util.function.Supplier;

public class ConstantPivotCommand<T extends TagalongSubsystemBase & PivotAugment> extends Command {

    private final TagalongPivot _pivot;
    private final double _lowerBound;
    private final double _upperBound;
    private double goalRot; // private final double _goalPos// itionRot;
    private double _maxVelocity;

    private boolean _startedMovement;
    private double _goalVelocityRPS = 0.0;


    public ConstantPivotCommand(
            T pivot,
            Supplier<Double> goalPosition,
            double maxVelocityRPS,
            double lowerToleranceRot,
            double upperToleranceRot
    ) {
        _pivot = pivot.getPivot();
        goalRot = goalPosition.get();
        _maxVelocity = maxVelocityRPS;
        _lowerBound = MathUtils.cppMod(goalRot, 1.0) - Math.abs(lowerToleranceRot);
        _upperBound = MathUtils.cppMod(goalRot, 1.0) + Math.abs(upperToleranceRot);

        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        _pivot.setHoldPivotPosition(false);

    }

    @Override
    public void execute() {
        _pivot.setPivotProfile(goalRot, _goalVelocityRPS, _maxVelocity);


    }

    @Override
    public void end(boolean interrupted) {

        _pivot.setPivotProfile(ShooterPivotAngles.STABLE.getRotations(), 0, .5);
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.commandXboxController.leftTrigger().getAsBoolean();
    }


}
