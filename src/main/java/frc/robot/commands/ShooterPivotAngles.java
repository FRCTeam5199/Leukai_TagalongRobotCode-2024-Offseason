package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.tagalong.TagalongAngle;

public enum ShooterPivotAngles implements TagalongAngle {
    STABLE(12),
    LOW(25),
    MID(30),
    MAX(52);
    private Rotation2d value;

    ShooterPivotAngles(double degree) {
        value = Rotation2d.fromDegrees(degree);
    }

    ShooterPivotAngles(Rotation2d value) {
        this.value = value;
    }

    @Override
    public double getDegrees() {
        return value.getDegrees();
    }

    @Override
    public double getRadians() {
        return value.getRadians();
    }

    @Override
    public double getRotations() {
        return value.getRotations();
    }
}
