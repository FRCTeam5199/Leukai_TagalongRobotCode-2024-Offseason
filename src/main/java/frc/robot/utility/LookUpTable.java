package frc.robot.utility;

import edu.wpi.first.math.Pair;

import java.util.ArrayList;

public class LookUpTable {
    public static ArrayList<Pair<Double, Double>> lookUpTable = new ArrayList<>() {
        {
            //Distances, Averages
            add(new Pair<>(1.37, 57d));
            add(new Pair<>(1.92, 46.75));
            add(new Pair<>(2.32, 43.125));
            add(new Pair<>(2.5, 38.85));
            add(new Pair<>(2.75, 36.95));
            add(new Pair<>(3.00, 35.55));
            add(new Pair<>(3.25, 33.30));
            add(new Pair<>(3.5, 32.50));
            add(new Pair<>(4.00, 30.65));
            add(new Pair<>(4.25, 29.20));
            add(new Pair<>(4.5, 27.1));
            add(new Pair<>(4.52, 27.095));
            add(new Pair<>(4.75, 26.15));
            add(new Pair<>(4.81, 26.15));
            add(new Pair<>(5.01, 25.675));
            add(new Pair<>(5.25, 24.4));
            add(new Pair<>(5.5, 24.35));
            add(new Pair<>(5.57, 24.3));
            add(new Pair<>(5.81, 23.8));
        }
    };

    /*Distances, Low Value, High Value
    1.92, 45.5, 49
    2.32, 42.5, 44.75
    2.5, 36.7, 41
    2.75, 35, 38.9
    3.00, 34.3, 36.8
    3.25, 32.2, 34.4
    3.5,  31.7 , 33.3
    3.75, 30, 31.3
    4.00, 29 , 30
    4.25, 28.5, 29.9
    ------70 RPS-------
    4.5,26.8, 27.4

    4.52, 26.25, 28
    4.75,    25.9   ,26.4
    4.81, 25.75, 26.55
    5.01, 25, 26.55
    5.25, 24, 25.25
    5.57, 24.5, 25
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
