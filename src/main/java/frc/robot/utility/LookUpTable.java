package frc.robot.utility;

import edu.wpi.first.math.Pair;

import java.util.ArrayList;

public class LookUpTable {
    public static ArrayList<Pair<Double, Double>> lookUpTable = new ArrayList<>() {
        {
            //Distances, Averages

            add(new Pair<>(1.37, 58d));
            add(new Pair<>(2.32, 47.75));
            add(new Pair<>(2.32, 44.625));
            add(new Pair<>(2.75, 39.75));
            add(new Pair<>(3.15, 36.5));
            add(new Pair<>(3.44, 34.75));
            add(new Pair<>(3.76, 34d));
            add(new Pair<>(4.14, 32d));
            add(new Pair<>(4.77, 30.625));
            add(new Pair<>(5.43, 28.75));
        }
    };

    /*Distances, Low Value, High Value
    1.92, 45.5, 50
    2.32, 43, 46.25
    2.75, 38, 41.5
    3.15, 35, 38
    3.46, 34, 35.5
    3.76, 33.5, 34.5
    4.14, 31.5, 32.5
    4.77, 30.5, 30.75

     */
    public static double findValue(double distance) {
        if (distance < lookUpTable.get(0).getFirst()) {
            return lookUpTable.get(0).getSecond();
        }
        if (distance > lookUpTable.get(lookUpTable.size() - 1).getFirst()) {
            return lookUpTable.get(lookUpTable.size() - 1).getSecond();
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
