package frc.robot.utility;

import edu.wpi.first.math.Pair;

import java.util.ArrayList;

public class LookUpTable {
    public static ArrayList<Pair<Double, Double>> lookUpTable = new ArrayList<>() {
        {
            //Distances, Averages
            add(new Pair<>(2.32, 44.625));
            add(new Pair<>(2.75, 39.75));
            add(new Pair<>(3.15, 36.5));
            add(new Pair<>(3.47, 34.25));
            add(new Pair<>(2.32, 33d));
        }
    };

    /*Distances, Low Value, High Value
    2.32, 43, 46.25
    2.75, 38, 41.5
    3.15, 35, 38
    3.47, 33, 35.5
    3.92, 32, 33.75


     */
    public static double findValue(double distance) {
        if (lookUpTable.size() < 2) {
            return 30d / 360d;
        }
        Pair<Double, Double> lowAutoAimValue = lookUpTable.get(0);
        Pair<Double, Double> highAutoAimValue = lookUpTable.get(lookUpTable.size() - 1);
        for (int i = 0; i < lookUpTable.size() - 1; i++) {
            if (distance > lookUpTable.get(i).getFirst() && distance < lookUpTable.get(i + 1).getFirst()) {
                lowAutoAimValue = lookUpTable.get(i);
                highAutoAimValue = lookUpTable.get(i + 1);
                break;
            }
        }

        double percentInBetween = (distance - lowAutoAimValue.getFirst()) / (highAutoAimValue.getFirst() - lowAutoAimValue.getFirst());

        return lowAutoAimValue.getSecond()
                - (percentInBetween * (lowAutoAimValue.getSecond() - highAutoAimValue.getSecond()));
    }
}
