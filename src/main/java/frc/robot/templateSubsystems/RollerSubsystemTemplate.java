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
import frc.robot.utility.PID;


public class RollerSubsystemTemplate extends SubsystemBase {
    private final TalonFX motor;
    private final TalonFXConfiguration motorConfig;
    private final TrapezoidProfile profile;
    private final PositionVoltage positionVoltage;
    private final VelocityVoltage velocityVoltage;
    private final Slot0Configs slot0Configs;
    private final SimpleMotorFeedforward feedforward;
    private final double lowerToleranceRollerRot;
    private final double upperToleranceRollerRot;
    private final Timer timer;
    private TrapezoidProfile.State currentState;
    private TrapezoidProfile.State goalState;
    private NeutralModeValue defaultNeutralMode;
    private double gearRatio = 1d;
    private boolean followLastRollerProfile = false;

    public RollerSubsystemTemplate(int id, TrapezoidProfile.Constraints constraints,
                                   PID pid, SimpleMotorFeedforward feedforward,
                                   double lowerToleranceRollerRot, double upperToleranceRollerRot,
                                   double[][] gearRatios) {
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

        this.lowerToleranceRollerRot = lowerToleranceRollerRot;
        this.upperToleranceRollerRot = upperToleranceRollerRot;

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

    public double getMotorRot() {
        return motor.getPosition().getValueAsDouble();
    }

    public double getRollerRot() {
        return motor.getPosition().getValueAsDouble() * gearRatio;
    }

    public double getDegreesFromRollerRot(double rollerRot) {
        return rollerRot * gearRatio * 360d;
    }

    public double getDegreesFromMotorRot(double motorRot) {
        return motorRot * 360d;
    }

    public double getRollerRotFromDegrees(double degrees) {
        return degrees / 360d / gearRatio;
    }

    public double getMotorRotFromDegrees(double degrees) {
        return degrees / 360d;
    }

    public double getRollerRotFromMotorRot(double motorRot) {
        return motorRot * gearRatio;
    }

    public double getMotorRotFromRollerRot(double rollerRot) {
        return rollerRot / gearRatio;
    }

    public double getMotorVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    public double getRollerVelocity() {
        return getRollerRotFromMotorRot(motor.getVelocity().getValueAsDouble());
    }

    public void setRollerVelocity(double rps) {
        followLastRollerProfile = false;
        motor.setNeutralMode(NeutralModeValue.Coast);
        motor.setControl(velocityVoltage.withVelocity(rps).withFeedForward(
                feedforward.calculate(rps, 0)
        ));
    }

    public void setRollerPower(double percent) {
        motor.set(percent);
    }

    public void setRollerProfile(double goalRot, double goalVelocity) {
        motor.setNeutralMode(NeutralModeValue.Coast);
        followLastRollerProfile = true;

        goalState.position = getMotorRotFromRollerRot(goalRot);
        goalState.velocity = goalVelocity;

        currentState = profile.calculate(0, currentState, goalState);
        currentState.position = getMotorRot();
        timer.restart();
    }

    public void followLastRollerProfile() {
        TrapezoidProfile.State nextState = profile.calculate(timer.get(), currentState, goalState);
        motor.setControl(
                positionVoltage.withPosition(nextState.position)
                        .withFeedForward(feedforward.calculate(nextState.velocity,
                                (nextState.velocity - currentState.velocity) / timer.get())));

        currentState = nextState;
        timer.restart();
    }

    public boolean isProfileFinished() {
        return currentState.position == goalState.position && currentState.velocity == goalState.velocity;
    }

    public boolean isRollerAtGoalState() {
        return isProfileFinished() &&
                getRollerRot() > getRollerRotFromMotorRot(goalState.position) - lowerToleranceRollerRot
                && getRollerRot() < getRollerRotFromMotorRot(goalState.position) + upperToleranceRollerRot;
    }

    @Override
    public void periodic() {
        if (followLastRollerProfile) followLastRollerProfile();
        if (isRollerAtGoalState()) {
            setRollerPower(0d);
            followLastRollerProfile = false;
        }
        if (getMotorVelocity() == 0 && getMotorRot() != 0) {
            motor.setNeutralMode(defaultNeutralMode);
            motor.setPosition(0d);
        }
    }
}
