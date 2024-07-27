package frc.robot.utility;

import java.util.ArrayList;

public class LookUpTable {
    public static ArrayList<AutoAimValue> listOfValues = new ArrayList<>() {
        {
            //add AutoAimValues here
        }
    };

    public static AutoAimValue findValue(double distance) {
        AutoAimValue lowAutoAimValue = listOfValues.get(0);
        AutoAimValue highAutoAimValue = listOfValues.get(listOfValues.size() - 1);
        for (int i = 0; i < listOfValues.size() - 1; i++) {
            if (distance > listOfValues.get(i).distance && distance < listOfValues.get(i + 1).distance) {
                lowAutoAimValue = listOfValues.get(i);
                highAutoAimValue = listOfValues.get(i + 1);
                break;
            }
        }

        double percentInBetween = (distance - lowAutoAimValue.distance) / (highAutoAimValue.distance - lowAutoAimValue.distance);

        double armAngle = lowAutoAimValue.armAngle - (percentInBetween * (lowAutoAimValue.armAngle - highAutoAimValue.armAngle));
        double shooterRPM = lowAutoAimValue.shooterRPM + (percentInBetween * (highAutoAimValue.shooterRPM - lowAutoAimValue.shooterRPM));

        return new AutoAimValue(distance, armAngle, shooterRPM);
    }
}
