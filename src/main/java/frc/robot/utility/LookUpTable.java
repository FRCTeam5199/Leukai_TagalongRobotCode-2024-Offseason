package frc.robot.utility;

import edu.wpi.first.math.Pair;

import java.util.ArrayList;

public class LookUpTable {
    public static ArrayList<Pair<Double, Double>> lookUpTable = new ArrayList<>() {
        {
            //Distances, Averages
            add(new Pair<>(1.37, 57d));
            add(new Pair<>(1.50, 52.25));
            add(new Pair<>(1.75, 49d));
            add(new Pair<>(1.92, 46.25));
            add(new Pair<>(2.32, 42.125));
            add(new Pair<>(2.5, 40.25));
            add(new Pair<>(2.75, 37.75));
            add(new Pair<>(3.00, 35.50));
            add(new Pair<>(3.25, 34d));
            add(new Pair<>(3.5, 32.60));
            add(new Pair<>(3.75, 31.25));
            add(new Pair<>(4.00, 30.1));
            add(new Pair<>(4.23, 29.05));
            add(new Pair<>(4.5, 27d));
            add(new Pair<>(4.52, 26.87));
            add(new Pair<>(4.75, 24.5));
            add(new Pair<>(4.82, 24.5d));
            add(new Pair<>(5.01, 24d));
            add(new Pair<>(5.25, 23.875));
            add(new Pair<>(5.5, 23.875));
            add(new Pair<>(5.57, 24.07));
            add(new Pair<>(5.81, 24.00));
        }
    };

    /*Distances, Low Value, High Value
    1.50, 49.5, 57
    1.75, 46, 52
    1.92, 43.5, 49
    2.32, 39.5, 44.75
    2.5, 38.5, 42
    2.75, 35.5, 40
    3.00, 34, 37
    3.25, 33, 35
    3.5,  32 , 33.2
    3.75, 31, 31.5
    4.00, 29 , 30
    4.25, 28.5, 29.9
    4.5,26.8, 27.426.
    ------70 RPS-------
    4.52, 26.25, 28
    4.75, 25.9, 26.4
    4.82, 23.75, 25.25
    5.01, 25, 26.55
    5.25, 24, 25.25
    5.57, 24.5, 25   */
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

//        return 1.689 * Math.pow(distance, 2) - 19.09 * distance + 78.07;
    }
}
