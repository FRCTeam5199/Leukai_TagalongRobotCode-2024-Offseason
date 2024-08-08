// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.parsers.IndexerParser;
import frc.robot.subsystems.minor.TagalongRoller;
import frc.robot.tagalong.RollerAugment;
import frc.robot.tagalong.TagalongSubsystemBase;

public class IndexerSubsystem extends TagalongSubsystemBase implements RollerAugment {
    private static IndexerSubsystem indexerSubsystem;
    private final TagalongRoller intake;
    private final TagalongRoller ampTrap;
    private final TagalongRoller indexer;
    private final IndexerParser indexerParser;
    private boolean isIndexerSubsystemDisabled = false;
    private boolean ampMode = false;
    private AnalogInput intakeSensor;
    private AnalogInput ampTrapSensor;
    private AnalogInput indexerSensor;

    public IndexerSubsystem(String filePath) {
        this(filePath == null ? null : new IndexerParser(Filesystem.getDeployDirectory(), filePath));
    }

    public IndexerSubsystem(IndexerParser parser) {
        super(parser);
        indexerParser = parser;

        intakeSensor = new AnalogInput(RollerConstants.INTAKE_SENSOR_ID);
        ampTrapSensor = new AnalogInput(RollerConstants.AMPTRAP_SENSOR_ID);
        indexerSensor = new AnalogInput(RollerConstants.INDEXER_SENSOR_ID);

        if (isIndexerSubsystemDisabled) {
            intake = new TagalongRoller(null);
            ampTrap = new TagalongRoller(null);
            indexer = new TagalongRoller(null);
            return;
        }
        intake = new TagalongRoller(indexerParser.intakeSideParser);
        ampTrap = new TagalongRoller(indexerParser.ampTrapParser);
        indexer = new TagalongRoller(indexerParser.shooterSideParser);

        configShuffleboard();
    }

    public static IndexerSubsystem getInstance() {
        if (indexerSubsystem == null) {
            indexerSubsystem = new IndexerSubsystem("configs/indexer/indexerConf.json");
        }
        return indexerSubsystem;
    }

    public boolean getAmpMode() {
        return ampMode;
    }

    public void setAmpMode(boolean ampMode) {
        this.ampMode = ampMode;
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
            case 1:
                return ampTrap;
            case 2:
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

    public void setRollerSpeeds(double intakeSpeed, double ampTrapSpeed, double indexerSpeed) {
        intake.setRollerVelocityControl(intakeSpeed, true);
        ampTrap.setRollerVelocityControl(ampTrapSpeed, true);
        indexer.setRollerVelocityControl(indexerSpeed, true);
    }

    public void setRollerProfiles(double intakeRotations, double ampTrapRotations, double indexerRotations) {
        intake.setRollerProfile(intakeRotations, 0);
        ampTrap.setRollerProfile(ampTrapRotations, 0);
        indexer.setRollerProfile(indexerRotations, 0);
    }

    public void setRollerPowers(double intakePercent, double ampTrapPercent, double indexerPercent) {
        intake.setRollerPower(intakePercent);
        ampTrap.setRollerPower(ampTrapPercent);
        indexer.setRollerPower(indexerPercent);
    }

    public void followLastRollerProfiles() {
        intake.followLastRollerProfile();
        ampTrap.followLastRollerProfile();
        indexer.followLastRollerProfile();
    }

    public double[] getRollerPositions() {
        return new double[]{
                intake.getRollerPosition(),
                ampTrap.getRollerPosition(),
                indexer.getRollerPosition()
        };
    }

    public boolean isRollerProfilesFinished() {
        return intake.isRollerProfileFinished() && ampTrap.isRollerProfileFinished() && indexer.isRollerProfileFinished();
    }

    public boolean isNoteInIntake() {
        return intakeSensor.getValue() < 100;
    }

    public boolean isNoteInAmpTrap() {
        return ampTrapSensor.getValue() < 100;
    }

    public boolean isNoteInIndexer() {
        return indexerSensor.getValue() < 100;
    }

    public static final class RollerConstants {
        public static final int INTAKE_SENSOR_ID = 2;
        public static final int AMPTRAP_SENSOR_ID = 1;
        public static final int INDEXER_SENSOR_ID = 3;
    }
}
