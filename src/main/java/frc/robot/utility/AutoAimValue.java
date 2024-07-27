package frc.robot.utility;

public class AutoAimValue {
    public double distance;
    public double armAngle;
    public double shooterRPM;

    public AutoAimValue(double distance, double armAngle, double shooterRPM) {
        this.distance = distance;
        this.armAngle = armAngle;
        this.shooterRPM = shooterRPM;
    }
}
