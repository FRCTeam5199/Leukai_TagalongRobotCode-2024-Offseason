// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.parsers.ClimberParser;
import frc.robot.parsers.IndexerParser;
import frc.robot.subsystems.minor.TagalongRoller;
import frc.robot.tagalong.RollerAugment;
import frc.robot.tagalong.TagalongSubsystemBase;

public class IndexerSubsystem extends TagalongSubsystemBase implements RollerAugment {
    public static final class RollerConstants {
        public static final int INTAKE_ID = 0;
        public static final int AMPTRAP_ID = 0;
        public static final int INDEXER_ID = 0;
        public static final int INTAKE_SENSOR_ID = 2;
        public static final int AMPTRAP_SENSOR_ID = 1;
        public static final int INDEXER_SENSOR_ID = 3;
    }

    private final TagalongRoller intake;
    private final TagalongRoller ampTrap;
    private final TagalongRoller indexer;
    private boolean isIndexerSubsystemDisabled = false;
    private final IndexerParser indexerParser;

    public IndexerSubsystem(String filePath) {
        this(filePath == null ? null : new IndexerParser(Filesystem.getDeployDirectory(), filePath));
    }

    public IndexerSubsystem(IndexerParser parser) {
        super(parser);
        indexerParser = parser;

        if (isIndexerSubsystemDisabled) {
            intake = new TagalongRoller(null);
            ampTrap = new TagalongRoller(null);
            indexer = new TagalongRoller(null);
            return;
        }
        intake = new TagalongRoller(indexerParser.intakeSideParser);
        ampTrap = new TagalongRoller(indexerParser.amptrapParser);
        indexer = new TagalongRoller(indexerParser.shooterSideParser);

        configShuffleboard();
    }

    @Override
    public void onEnable() {
        if (isIndexerSubsystemDisabled) {
            return;
        }
        intake.onEnable();
        ampTrap.onEnable();
        indexer.onEnable();
    }

    @Override
    public void onDisable() {
        if (isIndexerSubsystemDisabled) {
            return;
        }
        intake.onDisable();
        ampTrap.onDisable();
        indexer.onDisable();
    }

    @Override
    public void periodic() {
        if (isIndexerSubsystemDisabled) {
            return;
        }
        intake.periodic();
        ampTrap.periodic();
        indexer.periodic();

        updateShuffleboard();
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

    public void updateShuffleboard() {
        intake.updateShuffleboard();
        ampTrap.updateShuffleboard();
        indexer.updateShuffleboard();
    }

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
