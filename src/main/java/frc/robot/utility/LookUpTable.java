package frc.robot.utility;

import edu.wpi.first.math.Pair;

import java.util.ArrayList;

public class LookUpTable {
    public static ArrayList<Pair<Double, Double>> lookUpTableArmAngle = new ArrayList<>() {
        {
            //Distances, Averages
            add(new Pair<>(1.35, 57d));
            add(new Pair<>(1.6, 53d));
            add(new Pair<>(1.83, 47.0));
            add(new Pair<>(2.34, 41d));
            add(new Pair<>(2.75, 36.5));
            add(new Pair<>(3.1, 34d));
            add(new Pair<>(3.28, 31.8));
            add(new Pair<>(3.48, 29.5));
            add(new Pair<>(3.53, 29.3d));
            add(new Pair<>(4.064, 29d));
            add(new Pair<>(4.5, 26.75));
            add(new Pair<>(4.5, 25d));
            add(new Pair<>(5d, 23d));
        }
    };

    public static ArrayList<Pair<Double, Double>> lookUpTableDriveOffsetAngle = new ArrayList<>() {
        {
            //Y Distances, Angle
            add(new Pair<>(1.00, 0d));
            add(new Pair<>(1.35, 0d));
            add(new Pair<>(2.5, 0d));
            add(new Pair<>(3d, 2.5));
            add(new Pair<>(5d, 5d));
        }
    };


    /*
     * 4.5, 26, 27.5
     * 70 RPS
     * 4.5, 24.5, 25.5
     * 5, 23
     */

    public static double findArmAngle(double distance) {
        if (distance < lookUpTableArmAngle.get(0).getFirst()) {
            return lookUpTableArmAngle.get(0).getSecond();
        }
        if (distance > lookUpTableArmAngle.get(lookUpTableArmAngle.size() - 1).getFirst()) {
            return lookUpTableArmAngle.get(lookUpTableArmAngle.size() - 1).getSecond();
        }
        Pair<Double, Double> lowAutoAimValue = lookUpTableArmAngle.get(0);
        Pair<Double, Double> highAutoAimValue = lookUpTableArmAngle.get(lookUpTableArmAngle.size() - 1);
        for (int i = 0; i < lookUpTableArmAngle.size() - 1; i++) {
            if (distance > lookUpTableArmAngle.get(i).getFirst() && distance < lookUpTableArmAngle.get(i + 1).getFirst()) {
                lowAutoAimValue = lookUpTableArmAngle.get(i);
                highAutoAimValue = lookUpTableArmAngle.get(i + 1);
                break;
            }
        }

        double percentInBetween = (distance - lowAutoAimValue.getFirst()) / (highAutoAimValue.getFirst() - lowAutoAimValue.getFirst());

        return lowAutoAimValue.getSecond()
                - (percentInBetween * (lowAutoAimValue.getSecond() - highAutoAimValue.getSecond()));

//        return 1.689 * Math.pow(distance, 2) - 19.09 * distance + 78.07;
    }

    public static double findDriveOffsetAngle(double distance) {
        if (distance < lookUpTableDriveOffsetAngle.get(0).getFirst()) {
            return lookUpTableDriveOffsetAngle.get(0).getSecond();
        }
        if (distance > lookUpTableDriveOffsetAngle.get(lookUpTableDriveOffsetAngle.size() - 1).getFirst()) {
            return lookUpTableDriveOffsetAngle.get(lookUpTableDriveOffsetAngle.size() - 1).getSecond();
        }
        Pair<Double, Double> lowAutoAimValue = lookUpTableDriveOffsetAngle.get(0);
        Pair<Double, Double> highAutoAimValue = lookUpTableDriveOffsetAngle.get(lookUpTableDriveOffsetAngle.size() - 1);
        for (int i = 0; i < lookUpTableDriveOffsetAngle.size() - 1; i++) {
            if (distance > lookUpTableDriveOffsetAngle.get(i).getFirst() && distance < lookUpTableDriveOffsetAngle.get(i + 1).getFirst()) {
                lowAutoAimValue = lookUpTableDriveOffsetAngle.get(i);
                highAutoAimValue = lookUpTableDriveOffsetAngle.get(i + 1);
                break;
            }
        }

        double percentInBetween = (distance - lowAutoAimValue.getFirst()) / (highAutoAimValue.getFirst() - lowAutoAimValue.getFirst());

        return lowAutoAimValue.getSecond()
                - (percentInBetween * (lowAutoAimValue.getSecond() - highAutoAimValue.getSecond()));

//        return 1.689 * Math.pow(distance, 2) - 19.09 * distance + 78.07;
    }

//    add(new Pair<>(1.37, 57d));
//    add(new Pair<>(1.50, 52.25));
//    add(new Pair<>(1.75, 49d));
//    add(new Pair<>(1.92, 46.25));
//    add(new Pair<>(2.32, 42.125));
//    add(new Pair<>(2.5, 40.25));
//    add(new Pair<>(2.75, 37.75));
//    add(new Pair<>(3.00, 35.50));
//    add(new Pair<>(3.25, 34d));
//    add(new Pair<>(3.5, 32.60));
//    add(new Pair<>(3.75, 31.25));
//    add(new Pair<>(4.00, 30.1));
//    add(new Pair<>(4.23, 29.05));
//    add(new Pair<>(4.5, 27d));
//    add(new Pair<>(4.52, 26.87));
//    add(new Pair<>(4.75, 24.5));
//    add(new Pair<>(4.82, 24.5d));
//    add(new Pair<>(5.01, 24d));
//    add(new Pair<>(5.25, 23.875));
//    add(new Pair<>(5.5, 23.875));
//    add(new Pair<>(5.57, 24.07));
//    add(new Pair<>(5.81, 24.00));
//
}
