package frc.robot.automation;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutomationSelector {

    public AutomationSelector() {
        // Publish initial values to SmartDashboard
        SmartDashboard.putNumber("Reef Side", 1); // 1-6 counter clockwise. see strategy docs.
        SmartDashboard.putNumber("Position", 1); // 0 for left, 1 for middle, 2 for right
        SmartDashboard.putNumber("Level", 1); // L1-L3, 1-3
        SmartDashboard.putNumber("HumanPlayer", 0); // 0 for left, 1 for right
    }

    public int getReefSide() {
        // Retrieve value from SmartDashboard
        int value = (int) SmartDashboard.getNumber("Reef Side", 1);
        System.out.println("Retrieved Reef Side: " + value);
        return value;
    }

    public int getPosition() {
        // Retrieve value from SmartDashboard
        int value = (int) SmartDashboard.getNumber("Position", 1);
        System.out.println("Retrieved Position: " + value);
        return value;
    }

    public int getHeight() {
        // Retrieve value from SmartDashboard
        int value = (int) SmartDashboard.getNumber("Level", 1);
        System.out.println("Retrieved Level: " + value);
        return value;
    }

    public int getHumanPlayerStation() {
        // Retrieve value from SmartDashboard
        int value = (int) SmartDashboard.getNumber("HumanPlayer", 0);
        System.out.println("Retrieved HP: " + value);
        return value;
    }

}