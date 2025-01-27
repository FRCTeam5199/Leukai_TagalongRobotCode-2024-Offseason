// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.base;

import frc.robot.tagalong.TagalongHeight;

/**
 * Add your docs here.
 */
public enum ClimberHeights implements TagalongHeight {
    DOWN(0),
    UP_LEFT(1.61),
    UP_RIGHT(1.66);
    private double height;

    ClimberHeights(double height) {
        this.height = height;
    }

    @Override
    public double getHeightM() {
        return height;
    }
}
