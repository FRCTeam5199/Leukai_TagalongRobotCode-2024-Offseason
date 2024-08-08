package frc.robot.templateSubsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.utility.PID;


public class RollerSubsystemTemplate extends SubsystemBase {
    private final TalonFX motor;
    private final TalonFXConfiguration motorConfig;
    private final TrapezoidProfile profile;
    private final PositionVoltage positionVoltage;
    private final VelocityVoltage velocityVoltage;
    private final Slot0Configs slot0Configs;
    private final SimpleMotorFeedforward feedforward;
    private final double lowerTolerance;
    private final double upperTolerance;
    private final Timer timer;
    private TrapezoidProfile.State currentState;
    private TrapezoidProfile.State goalState;
    private NeutralModeValue defaultNeutralMode;
    private double gearRatio = 1d;
    private boolean followLastRollerProfile = false;

    public RollerSubsystemTemplate(int id, TrapezoidProfile.Constraints constraints,
                                   PID pid, SimpleMotorFeedforward feedforward,
                                   double lowerTolerance, double upperTolerance, double[][] gearRatios) {
        motor = new TalonFX(id);
        motorConfig = new TalonFXConfiguration();

        profile = new TrapezoidProfile(constraints);
        goalState = new TrapezoidProfile.State(0.0, 0.0);
        currentState = new TrapezoidProfile.State(0.0, 0.0);

        slot0Configs = pid.getSlot0Configs();
        motor.getConfigurator().apply(slot0Configs);

        this.feedforward = feedforward;

        positionVoltage = new PositionVoltage(0).withSlot(0).withEnableFOC(true);
        velocityVoltage = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);

        this.lowerTolerance = lowerTolerance;
        this.upperTolerance = upperTolerance;

        for (double[] ratio : gearRatios) {
            this.gearRatio *= ratio[1] / ratio[0];
        }

        timer = new Timer();
    }

    public void configureMotor(boolean isInverted, boolean isBrakeMode, double supplyCurrentLimit, double statorCurrentLimit) {
        defaultNeutralMode = isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        motorConfig.MotorOutput.Inverted =
                isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = defaultNeutralMode;
        motorConfig.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
        motorConfig.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;

        motor.getConfigurator().apply(motorConfig);
    }

    public double getDegrees() {
        return motor.getPosition().getValueAsDouble() * gearRatio / 360d;
    }

    public double getDegreesFromRotations(double rotations) {
        return rotations * gearRatio * 360d;
    }

    public double getRotations() {
        return motor.getPosition().getValueAsDouble();
    }

    public double getRotationsFromDegrees(double degrees) {
        return degrees / 360d / gearRatio;
    }

    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    public void setRollerPower(double percent) {
        motor.set(percent);
    }

    public void setRollerVelocity(double rps) {
        followLastRollerProfile = false;
        motor.setControl(velocityVoltage.withVelocity(rps).withFeedForward(
                feedforward.calculate(rps, 0)
        ));
    }

    public void setRollerProfile(double goalAngle, double goalVelocity) {
        motor.setNeutralMode(NeutralModeValue.Coast);
        followLastRollerProfile = true;

        goalState.position = getRotationsFromDegrees(goalAngle);
        goalState.velocity = goalVelocity;

        currentState = profile.calculate(0, currentState, goalState);
        currentState.position = getRotations();
        timer.restart();
    }

    public void followLastRollerProfile() {
        TrapezoidProfile.State nextState = profile.calculate(timer.get(), currentState, goalState);
        motor.setControl(
                positionVoltage.withPosition(nextState.position)
                        .withFeedForward(feedforward.calculate(nextState.velocity,
                                (nextState.velocity - currentState.velocity) / timer.get())));

        currentState = nextState;
        System.out.println(goalState.position);
//        System.out.println(currentState.position);
        timer.restart();
    }

    public boolean isProfileFinished() {
        return currentState.position == goalState.position && currentState.velocity == goalState.velocity;
    }

    public boolean isRollerAtGoalState() {
        return isProfileFinished() &&
                getDegrees() > getDegreesFromRotations(goalState.position) - lowerTolerance
                && getDegrees() < getDegreesFromRotations(goalState.position) + upperTolerance;
    }

    @Override
    public void periodic() {
        if (followLastRollerProfile) followLastRollerProfile();
        if (getVelocity() == 0) motor.setNeutralMode(defaultNeutralMode);
    }
}
