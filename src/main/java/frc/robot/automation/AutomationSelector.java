package frc.robot.automation;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utils.CowboyUtils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutomationSelector {

    public AutomationSelector() {
        // Publish initial values to SmartDashboard
        SmartDashboard.putNumber("Reef Side", 1);
        SmartDashboard.putNumber("Position", 1);
        SmartDashboard.putNumber("Level", 1);
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
}