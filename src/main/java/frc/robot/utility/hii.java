package frc.robot.utility;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.minor.TagalongFlywheel;

public class hii {

    public static Orchestra orchestra = new Orchestra();
    private final TagalongFlywheel motor1 = new TagalongFlywheel(null);

    public hii() {
        orchestra.addInstrument(RobotContainer.commandSwerveDrivetrain.getModule(0).getSteerMotor());
        orchestra.addInstrument(RobotContainer.commandSwerveDrivetrain.getModule(1).getSteerMotor());
        orchestra.addInstrument(RobotContainer.commandSwerveDrivetrain.getModule(2).getSteerMotor());
        orchestra.addInstrument(RobotContainer.commandSwerveDrivetrain.getModule(3).getSteerMotor());
        orchestra.addInstrument(RobotContainer.shooterSubsystem.getFlywheel(0).getFlywheelMotor());
        orchestra.addInstrument(RobotContainer.shooterSubsystem.getFlywheel(1).getFlywheelMotor());


    }

    public static void playTotikfr() {
        orchestra.loadMusic("totikfr.chrp");
        orchestra.play();
    }

    public static void playMegolovania() {
        orchestra.loadMusic("megolovania.chrp");
        orchestra.play();
    }

    public static void playCarelessWhisper() {
        orchestra.loadMusic("carelesswhisper.chrp");
        orchestra.play();
    }

}
