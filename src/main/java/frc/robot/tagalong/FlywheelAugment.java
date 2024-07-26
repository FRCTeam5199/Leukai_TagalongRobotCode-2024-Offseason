package frc.robot.tagalong;

import frc.robot.subsystems.minor.TagalongDualMotorFlywheel;

public interface FlywheelAugment {
  public TagalongDualMotorFlywheel getFlywheel();
  public TagalongDualMotorFlywheel getFlywheel(int i);
}
