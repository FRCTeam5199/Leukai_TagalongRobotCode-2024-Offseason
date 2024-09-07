package frc.robot.templateSubsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.PID;

public class FlywheelSubsystemTemplate extends SubsystemBase {
    private final TalonFX motor;
    private final TalonFXConfiguration motorConfig;
    private final TrapezoidProfile profile;
    private final VelocityVoltage velocityVoltage;
    private final Slot0Configs slot0Configs;
    private final SimpleMotorFeedforward feedforward;
    private final double lowerToleranceFlywheelVelocity;
    private final double upperToleranceFlywheelVelocity;
    private final Timer timer;
    private TrapezoidProfile.State currentState;
    private TrapezoidProfile.State goalState;
    private NeutralModeValue defaultNeutralMode;
    private double gearRatio = 1d;
    private boolean followLastFlywheelProfile = false;
    private double flywheelTargetVelocity = 0;

    public FlywheelSubsystemTemplate(int id, TrapezoidProfile.Constraints constraints,
                                     PID pid, SimpleMotorFeedforward feedforward,
                                     double lowerToleranceFlywheelVelocity, double upperToleranceFlywheelVelocity,
                                     double[][] gearRatios) {
        motor = new TalonFX(id);
        motorConfig = new TalonFXConfiguration();

        profile = new TrapezoidProfile(constraints);
        goalState = new TrapezoidProfile.State(0.0, 0.0);
        currentState = new TrapezoidProfile.State(0.0, 0.0);

        slot0Configs = pid.getSlot0Configs();
        motor.getConfigurator().apply(slot0Configs);

        this.feedforward = feedforward;

        velocityVoltage = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);

        this.lowerToleranceFlywheelVelocity = lowerToleranceFlywheelVelocity;
        this.upperToleranceFlywheelVelocity = upperToleranceFlywheelVelocity;

        for (double[] ratio : gearRatios) {
            this.gearRatio *= ratio[1] / ratio[0];
        }

        timer = new Timer();
    }

    public void configureMotor(boolean isInverted, double supplyCurrentLimit, double statorCurrentLimit) {
        motorConfig.MotorOutput.Inverted =
                isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
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

    public double getFlywheelRot() {
        return motor.getPosition().getValueAsDouble() * gearRatio;
    }

    public double getDegreesFromFlywheelRot(double flywheelRot) {
        return flywheelRot * gearRatio * 360d;
    }

    public double getDegreesFromMotorRot(double motorRot) {
        return motorRot * 360d;
    }

    public double getFlywheelRotFromDegrees(double degrees) {
        return degrees / 360d / gearRatio;
    }

    public double getMotorRotFromDegrees(double degrees) {
        return degrees / 360d;
    }

    public double getFlywheelRotFromMotorRot(double motorRot) {
        return motorRot * gearRatio;
    }

    public double getMotorRotFromFlywheelRot(double flywheelRot) {
        return flywheelRot / gearRatio;
    }

    public double getMotorVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    public double getFlywheelVelocity() {
        return getFlywheelRotFromMotorRot(motor.getVelocity().getValueAsDouble());
    }

    public void setFlywheelVelocity(double rps) {
        flywheelTargetVelocity = rps;
        motor.setControl(velocityVoltage.withVelocity(getMotorRotFromFlywheelRot(rps)).withFeedForward(
                feedforward.calculate(getMotorRotFromFlywheelRot(rps), 0)
        ));
    }

    public void setFlywheelPower(double percent) {
        motor.set(percent);
    }

    public boolean isFlywheelAtTargetSpeed() {
        return getFlywheelVelocity() >= flywheelTargetVelocity - lowerToleranceFlywheelVelocity
                && getFlywheelVelocity() <= flywheelTargetVelocity + upperToleranceFlywheelVelocity;
    }
}
