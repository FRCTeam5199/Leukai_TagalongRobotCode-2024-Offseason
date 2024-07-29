package frc.robot.utility;

import edu.wpi.first.math.Pair;

import java.util.ArrayList;

public class LookUpTable {
    public static ArrayList<Pair<Double, Double>> lookUpTable = new ArrayList<>() {
        {
            //add AutoAimValues here
        }
    };

    public static double findValue(double distance) {
        if (lookUpTable.size() < 2) {
            return 40;
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
