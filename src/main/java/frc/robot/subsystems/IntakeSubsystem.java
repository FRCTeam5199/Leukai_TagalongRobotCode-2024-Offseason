// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.subsystems.minor.TagalongRoller;
import frc.robot.tagalong.RollerAugment;
import frc.robot.tagalong.TagalongSubsystemBase;

public class IntakeSubsystem extends TagalongSubsystemBase implements RollerAugment {

    public IntakeSubsystem(Object parser) {
        super(parser);
        if (isRollerSubsystemDisabled) {
            intake = new TagalongRoller(null);
            ampTrap = new TagalongRoller(null);
            indexer = new TagalongRoller(null);
            return;
        }
        intake = new TagalongRoller(null);
        ampTrap = new TagalongRoller(null);
        indexer = new TagalongRoller(null);
    }

    public static final class RollerConstants {
        public static final int INTAKE_ID = 0;
        public static final int AMPTRAP_ID = 0;
        public static final int INDEXER_ID = 0;
        public static final int INTAKE_SENSOR_ID = 0;
        public static final int AMPTRAP_SENSOR_ID = 0;
        public static final int INDEXER_SENSOR_ID = 0;
    }

    private final TagalongRoller intake;
    private final TagalongRoller ampTrap;
    private final TagalongRoller indexer;
    private boolean isRollerSubsystemDisabled = false;

    @Override
    public void onEnable() {
        if (isRollerSubsystemDisabled) {
            return;
        }
        intake.onEnable();
        ampTrap.onEnable();
        indexer.onEnable();
    }

    @Override
    public void onDisable() {
        if (isRollerSubsystemDisabled) {
            return;
        }
        intake.onDisable();
        ampTrap.onDisable();
        indexer.onDisable();
    }

    @Override
    public void periodic() {
        if (isRollerSubsystemDisabled) {
            return;
        }
        intake.periodic();
        ampTrap.periodic();
        indexer.periodic();
    }

    public void disabledPeriodic() {
        intake.disabledPeriodic();
        ampTrap.disabledPeriodic();
        indexer.disabledPeriodic(); 
    }

    @Override
    public void simulationInit() {
        intake.simulationInit();
        ampTrap.simulationInit();
        indexer.simulationInit();
    }

    @Override
    public void simulationPeriodic() {
        intake.simulationPeriodic();
        ampTrap.simulationPeriodic();
        indexer.simulationPeriodic();
    }

    @Override
    public void updateShuffleboard() {
        intake.updateShuffleboard();
        ampTrap.updateShuffleboard();
        indexer.updateShuffleboard();
    }

    @Override
    public void configShuffleboard() {
        intake.configShuffleboard();
        ampTrap.configShuffleboard();
        indexer.configShuffleboard();
    }

    @Override
    public TagalongRoller getRoller() {
        return intake;
    }

    @Override
    public TagalongRoller getRoller(int i) {
        switch (i) {
            case 0:
                return intake;
            case 1:
                return ampTrap;
            case 3:
                return indexer;
            default:
                return intake;
        }
    }

    @Override
    public boolean checkInitStatus() {
        return super.checkInitStatus() && intake.checkInitStatus() 
            && ampTrap.checkInitStatus() && indexer.checkInitStatus();
    }
}
