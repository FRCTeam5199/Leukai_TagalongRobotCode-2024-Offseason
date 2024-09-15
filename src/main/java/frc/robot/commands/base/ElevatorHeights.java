// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.base;

import frc.robot.tagalong.TagalongHeight;

/**
 * Add your docs here.
 */
public enum ElevatorHeights implements TagalongHeight {
    STABLE(0),
    AMP(.448),
    TRAP(0.58);
    private double meters;

    ElevatorHeights(double meters) {
        this.meters = meters;
    }

    @Override
    public double getHeightM() {
        return meters;
    }
}
