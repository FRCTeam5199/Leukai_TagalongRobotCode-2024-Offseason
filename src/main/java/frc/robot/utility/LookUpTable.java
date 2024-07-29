package frc.robot.utility;

import edu.wpi.first.math.Pair;
import frc.robot.commands.ShooterPivotAngles;

import java.util.ArrayList;

public class LookUpTable {
    public static ArrayList<Pair<Double, Double>> listOfValues = new ArrayList<>() {
        {
            //add AutoAimValues here
        }
    };

    public static double findValue(double distance) {
        Pair<Double, Double> lowAutoAimValue = listOfValues.get(0);
        Pair<Double, Double> highAutoAimValue = listOfValues.get(listOfValues.size() - 1);
        for (int i = 0; i < listOfValues.size() - 1; i++) {
            if (distance > listOfValues.get(i).getFirst() && distance < listOfValues.get(i + 1).getFirst()) {
                lowAutoAimValue = listOfValues.get(i);
                highAutoAimValue = listOfValues.get(i + 1);
                break;
            }
        }

        double percentInBetween = (distance - lowAutoAimValue.getFirst()) / (highAutoAimValue.getFirst() - lowAutoAimValue.getFirst());

        return lowAutoAimValue.getSecond()
                - (percentInBetween * (lowAutoAimValue.getSecond() - highAutoAimValue.getSecond()));
    }
}
