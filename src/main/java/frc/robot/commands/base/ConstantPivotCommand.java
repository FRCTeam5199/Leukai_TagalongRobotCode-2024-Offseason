package frc.robot.commands.base;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.minor.TagalongPivot;
import frc.robot.tagalong.MathUtils;
import frc.robot.tagalong.PivotAugment;
import frc.robot.tagalong.TagalongSubsystemBase;

import java.util.function.Supplier;

public class ConstantPivotCommand<T extends TagalongSubsystemBase & PivotAugment> extends Command {
    private final TagalongPivot _pivot;
    private final boolean _holdPositionAfter;
    private final double _lowerBound;
    private final double _upperBound;
    private double _goalPositionRot; // private final double _goalPositionRot;
    private double _maxVelocity;

    private boolean _startedMovement;
    private double _goalVelocityRPS = 0.0;

    // private boolean midShotUpdate = false;
    // private double midShotUpdateGoal = 0d;

    public ConstantPivotCommand(
            T pivot, Supplier<Double> goalPosition, boolean holdPositionAfter, double maxVelocityRPS
    ) {
        this(
                pivot,
                goalPosition,
                holdPositionAfter,
                maxVelocityRPS,
                pivot._configuredDisable ? 0.0 : pivot.getPivot()._defaultPivotLowerToleranceRot,
                pivot._configuredDisable ? 0.0 : pivot.getPivot()._defaultPivotUpperToleranceRot
        );
    }

    public ConstantPivotCommand(
            T pivot,
            Supplier<Double> goalPosition,
            boolean holdPositionAfter,
            double maxVelocityRPS,
            double lowerToleranceRot,
            double upperToleranceRot
    ) {
        _pivot = pivot.getPivot();
        _goalPositionRot = goalPosition.get();
        _holdPositionAfter = holdPositionAfter;
        _maxVelocity = maxVelocityRPS;
        _lowerBound = MathUtils.cppMod(_goalPositionRot, 1.0) - Math.abs(lowerToleranceRot);
        _upperBound = MathUtils.cppMod(_goalPositionRot, 1.0) + Math.abs(upperToleranceRot);

        addRequirements(pivot);
    }

    public ConstantPivotCommand(
            TagalongPivot pivot,
            Supplier<Double> goalPosition,
            boolean holdPositionAfter
    ) {
        _pivot = pivot;
        _goalPositionRot = goalPosition.get();
        _holdPositionAfter = holdPositionAfter;
        _lowerBound = 0;
        _upperBound = 0;

        // addRequirements(pivot);
    }

    public ConstantPivotCommand(T pivot, Supplier<Double> goalPosition, boolean holdPositionAfter) {
        this(
                pivot,
                goalPosition,
                holdPositionAfter,
                pivot._configuredDisable ? 0.0 : pivot.getPivot()._maxVelocityRPS
        );
    }

    @Override
    public void initialize() {
        _pivot.setHoldPivotPosition(false);
        _startedMovement = false;
    }

    @Override
    public void execute() {
        if (!_startedMovement && _pivot.isSafeToMove()) {
            _startedMovement = true;
            _pivot.setPivotProfile(_goalPositionRot, _goalVelocityRPS, _maxVelocity);
        }

        // if(_startedMovement && midShotUpdate) {
        //     midShotUpdate = false;
        //     _goalPositionRot = midShotUpdateGoal;

        // }

        if (_startedMovement) {
            _pivot.followLastProfile();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // if (!interrupted) {
        _pivot.setHoldPivotPosition(_holdPositionAfter);
        // }

    }

    @Override
    public boolean isFinished() {
        return RobotContainer.commandXboxController.rightBumper().getAsBoolean();
    }
}
