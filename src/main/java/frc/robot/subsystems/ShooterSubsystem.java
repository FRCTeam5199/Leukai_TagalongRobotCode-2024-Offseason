package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.parsers.ShooterParser;
import frc.robot.subsystems.minor.TagalongFlywheel;
import frc.robot.subsystems.minor.TagalongPivot;
import frc.robot.tagalong.FlywheelAugment;
import frc.robot.tagalong.PivotAugment;
import frc.robot.tagalong.TagalongSubsystemBase;

public class ShooterSubsystem extends TagalongSubsystemBase implements PivotAugment, FlywheelAugment {
    private static ShooterSubsystem shooterSubsystem;
    public final ShooterParser shooterParser;
    private final TagalongFlywheel shooterLeft;
    private final TagalongFlywheel shooterRight;
    private final TagalongPivot arm;
    private boolean isShooterSubsystemDisabled = false;

    public ShooterSubsystem(String filePath) {
        this(filePath == null ? null : new ShooterParser(Filesystem.getDeployDirectory(), filePath));
    }

    public ShooterSubsystem(ShooterParser parser) {
        super(parser);
        shooterParser = parser;
        if (isShooterSubsystemDisabled) {
            shooterLeft = new TagalongFlywheel(null);
            shooterRight = new TagalongFlywheel(null);
            arm = new TagalongPivot(null);
            return;
        }
        shooterLeft = new TagalongFlywheel(shooterParser.flywheelLeftParser);
        shooterRight = new TagalongFlywheel(shooterParser.flywheelRightParser);
        arm = new TagalongPivot(shooterParser.pivotParser);

        configShuffleboard();
    }

    public static ShooterSubsystem getInstance() {
        if (shooterSubsystem == null) {
            shooterSubsystem = new ShooterSubsystem("configs/shooter/shooterConf.json");
        }
        return shooterSubsystem;
    }

    @Override
    public TagalongFlywheel getFlywheel() {
        return shooterLeft;
    }

    @Override
    public TagalongFlywheel getFlywheel(int i) {
        switch (i) {
            case 0:
                return shooterLeft;
            case 1:
                return shooterRight;
            default:
                return shooterLeft;
        }
    }

    @Override
    public TagalongPivot getPivot() {
        return arm;
    }

    @Override
    public TagalongPivot getPivot(int i) {
        return arm;
    }

    @Override
    public void onEnable() {
        shooterLeft.onEnable();
        shooterRight.onEnable();
        arm.onEnable();
    }

    @Override
    public void onDisable() {
        if (_isSubsystemDisabled) {
            return;
        }
        shooterLeft.onDisable();
        shooterRight.onDisable();
        arm.onDisable();
    }

    @Override
    public void periodic() {
        if (_isSubsystemDisabled) {
            return;
        }
        shooterLeft.periodic();
        shooterRight.periodic();
        arm.periodic();

        updateShuffleboard();
//        System.out.println("Reached Shooter Conditions: " + reachedShootingConditions(60));
    }

    @Override
    public void disabledPeriodic() {
        shooterLeft.disabledPeriodic();
        shooterRight.disabledPeriodic();
        arm.disabledPeriodic();
    }

    @Override
    public void simulationInit() {
        shooterLeft.simulationInit();
        shooterRight.simulationInit();
        arm.simulationInit();
    }

    @Override
    public void simulationPeriodic() {
        shooterLeft.simulationPeriodic();
        shooterRight.simulationPeriodic();
        arm.simulationPeriodic();
    }

    @Override
    public void updateShuffleboard() {
        shooterLeft.updateShuffleboard();
        shooterRight.updateShuffleboard();
        arm.updateShuffleboard();
    }

    @Override
    public void configShuffleboard() {
        shooterLeft.configShuffleboard();
        shooterRight.configShuffleboard();
        arm.configShuffleboard();
    }

    @Override
    public boolean checkInitStatus() {
        return super.checkInitStatus() && shooterLeft.checkInitStatus() && shooterRight.checkInitStatus()
                && arm.checkInitStatus();
    }

    public void moveShooterToSetpointAndSpeed(double setpoint, double targetSpeed) {
        arm.setPivotProfile(setpoint);

        shooterLeft.setFlywheelControl(targetSpeed, true);
        shooterRight.setFlywheelControl(.5 * targetSpeed, true);
    }

    public void followLastPivotProfile() {
        arm.followLastProfile();
    }

    public boolean reachedShootingConditions(double targetSpeed) {
        double percentageOfMaxSpeed = .99;
        return shooterLeft.getFlywheelMotor().getVelocity().getValueAsDouble() > targetSpeed * percentageOfMaxSpeed
                && shooterRight.getFlywheelMotor().getVelocity().getValueAsDouble() > targetSpeed * .5 * percentageOfMaxSpeed;
    }

    public boolean shotNote(double targetSpeed) {
        return ((shooterRight.getFlywheelVelocity() > (targetSpeed * 0.9)) && (shooterLeft.getFlywheelVelocity() > ((targetSpeed * 0.5) * 0.9)));
    }
}
