package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.UserInterface;
import frc.robot.commands.ScoreCommands;
import frc.robot.constants.Constants;
import frc.robot.parsers.ShooterParser;
import frc.robot.subsystems.minor.TagalongFlywheel;
import frc.robot.subsystems.minor.TagalongPivot;
import frc.robot.tagalong.FlywheelAugment;
import frc.robot.tagalong.PivotAugment;
import frc.robot.tagalong.TagalongSubsystemBase;
import frc.robot.utility.LookUpTable;

import java.sql.SQLOutput;

import static frc.robot.RobotContainer.commandSwerveDrivetrain;

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
        System.out.println("Shooter LEFT: " + shooterLeft.getFlywheelVelocity());
//        System.out.println("Shooter RIGHT: " + shooterRight.getFlywheelVelocity());
        // System.out.println(UserInterface.getInstance().getShooterPositionComponentData());
//        System.out.println("Arm Pivot Position: " + Rotation2d.fromRotations(arm.getPivotPosition()).getDegrees());
//        System.out.println("Reached Shooting Conditions: " + reachedShootingCondtions(60));
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
        arm.setPivotProfile(Rotation2d.fromDegrees(setpoint).getDegrees());

        if (targetSpeed == 0) {
            shooterSubsystem.setFlywheelPowers(0);
        } else {
            shooterLeft.setFlywheelControl(targetSpeed, true);
            shooterRight.setFlywheelControl(.57 * targetSpeed, true);
        }
    }

    public void followLastPivotProfile() {
        arm.followLastProfile();
    }

    public boolean reachedShootingCondtions(double targetSpeed) {
        double percentageOfMaxSpeed = .95;

        return shooterLeft.getFlywheelMotor().getVelocity().getValueAsDouble() >= targetSpeed * percentageOfMaxSpeed
                && shooterRight.getFlywheelMotor().getVelocity().getValueAsDouble() >= targetSpeed * .57 * percentageOfMaxSpeed;
    }

    public void setShooterSpeeds(double rps) {
        shooterLeft.setFlywheelControl(rps, true);
        shooterRight.setFlywheelControl(rps * .57, true);
    }

    public double getAutoAimAngle() {
        double distance;
        double[] robotCoords = new double[]{commandSwerveDrivetrain.getPose().getX(), commandSwerveDrivetrain.getPose().getY()};
        if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
            distance = ScoreCommands.getDistance(robotCoords, Constants.Vision.RED_SPEAKER_COORDINATES);
        else
            distance = ScoreCommands.getDistance(robotCoords, Constants.Vision.BLUE_SPEAKER_COORDINATES);

        double armAngle = LookUpTable.findValue(distance);
        return armAngle;
    }

    public void setFlywheelPowers(double percent) {
        shooterLeft.setFlywheelPower(percent);
        shooterRight.setFlywheelPower(percent);
    }
}
