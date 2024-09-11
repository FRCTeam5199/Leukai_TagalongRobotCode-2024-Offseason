package frc.robot.utility;

import edu.wpi.first.math.Pair;

import java.util.ArrayList;

public class LookUpTable {
    public static ArrayList<Pair<Double, Double>> lookUpTable = new ArrayList<>() {
        {

            add(new Pair<>(2.5, 38.85));
            add(new Pair<>(2.75, 36.95));
            add(new Pair<>(3.00, 35.55));
            add(new Pair<>(3.25, 33.30));
            add(new Pair<>(3.5, 32.50));
            add(new Pair<>(4.00, 30.65));
            add(new Pair<>(4.25, 29.20));



            //Distances, Averages
//            add(new Pair<>(1.37, 57d));
//            add(new Pair<>(1.92, 46.75));
//            add(new Pair<>(2.32, 43.125));
//            add(new Pair<>(2.75, 38.75));
//            add(new Pair<>(3.15, 36.5));
//            add(new Pair<>(3.4266, 32.875));
//            add(new Pair<>(3.87, 32.375));
//            add(new Pair<>(4.14, 31.75));
//            add(new Pair<>(4.4, 32.925));
//
//            add(new Pair<>(4.48, 30.875));
//            add(new Pair<>(4.52, 27.125));
//            add(new Pair<>(4.81, 26.25));
//            add(new Pair<>(5.01, 25.875));
//            add(new Pair<>(5.25, 24.4));
//            add(new Pair<>(5.57, 24.3));
        }

    };

    // 2.5, 36.7, 41
    // 2.75, 35, 38.9
    //3.00, 34.3, 36.8
    //3.25, 32.2, 34.4
    //3.5,  31.7 , 33.3
    //3.75, 30, 31.3
    //4.00, 29 , 30
    //4.25, 28.5, 29.9
    //4.5,

    /*Distances, Low Value, High Value
    ------60 RPS--------
    1.92, 45.5, 49
    2.32, 42.5, 44.75
    2.75, 38, 40.5
    3.15, 35, 38
    3.4266, 31, 33.5
    3.46, 34, 35.5
    3.87, 31.25, 33.5
    4.14, 31.25, 32.25
    4.48, 30.25, 31.5
    -------70 RPS-------
    4.52, 26.25, 28
    4.81, 25.75, 26.75
    5.01, 25, 26.75
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
