package frc.robot.automation;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants.ScoringConstants.Setpoints;

public class AutomationTabletInput {

    public AutomationTabletInput() {
        // Publish initial values to SmartDashboard
        SmartDashboard.putNumber("Reef Side", 1); // 1-6 counter clockwise. see strategy docs.
        SmartDashboard.putNumber("Position", 1); // 0 for left, 1 for middle, 2 for right
        SmartDashboard.putNumber("Level", 1); // L1-L3, 1-3
        SmartDashboard.putNumber("HumanPlayer", 0); // 0 for left, 1 for right
        SmartDashboard.putBoolean("HomeSubsystems", false);
        SmartDashboard.putBoolean("IntakeOn", false);
        SmartDashboard.putBoolean("OuttakeOn", false);
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

    public Setpoints getHeight() {
        // Retrieve value from SmartDashboard
        int value = (int) SmartDashboard.getNumber("Level", 1);
        System.out.println("Retrieved Level: " + value);

        switch (value) {
            case 1:
                return Setpoints.L1;
            case 2:
                return Setpoints.L2;
            case 3:
                return Setpoints.L3;
            default:
                return Setpoints.HOME; // Default to HOME if invalid value
        }
    }

    public int getHumanPlayerStation() {
        // Retrieve value from SmartDashboard
        int value = (int) SmartDashboard.getNumber("HumanPlayer", 0);
        System.out.println("Retrieved HP: " + value);
        return value;
    }
}