package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.limelight.LimelightHelpers;
import frc.robot.limelight.LimelightHelpers.LimelightResults;
import frc.robot.limelight.LimelightHelpers.LimelightTarget_Classifier;
import frc.robot.limelight.LimelightHelpers.LimelightTarget_Detector;

public class ObjectDetectionSubsystem extends SubsystemBase {
    private static ObjectDetectionSubsystem objectDetectionSubsystem;

    public static ObjectDetectionSubsystem getInstance(){
        if (objectDetectionSubsystem == null) objectDetectionSubsystem = new ObjectDetectionSubsystem();
        return objectDetectionSubsystem;
    }
    
    public LimelightResults getLimelightResults(){
        return LimelightHelpers.getLatestResults(Constants.Vision.LIMELIGHT);
    }

    public LimelightTarget_Classifier[] getTargets(){
        return getLimelightResults().targetingResults.targets_Classifier;
    }

    public LimelightTarget_Detector[] getObjects(){
        return getLimelightResults().targetingResults.targets_Detector;

    }

    public LimelightTarget_Detector getNearestNote() {
        LimelightTarget_Detector nearestNote = null;
        double minDistance = Double.MAX_VALUE;
    
        for (LimelightTarget_Detector object : getObjects()) {
            if (object.className.equals("note")) {
                double distance = calculateDistance(object); // Implement this method
                if (distance < minDistance) {
                    nearestNote = object;
                    minDistance = distance;
                }
            }
        }
    
        return nearestNote;
    }
    
    private double calculateDistance(LimelightTarget_Detector object) {
        // Calculate the distance to the object based on its coordinates (tx, ty)
        // Replace this with the actual distance calculation based on your needs
        return Math.sqrt(object.tx * object.tx + object.ty * object.ty);
    }

    public double getNotePoseX(){
        return getNearestNote().tx;
    }

    public double getNotePoseY(){
        return getNearestNote().ty;
    }
    
    public double getNoteDistance(){
        return getNearestNote().ta;
    }

    public String getObjectIdentity(){
        return getNearestNote().className;
    }


    