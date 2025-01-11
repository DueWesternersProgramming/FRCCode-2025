package frc.robot.automation;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.ScoringConstants;

public class AutomationSelector {

    SendableChooser<Integer> reefSideChooser = new SendableChooser<>();
    SendableChooser<Integer> positionChooser = new SendableChooser<>();
    SendableChooser<Integer> heightChooser = new SendableChooser<>();

    public AutomationSelector() {

        reefSideChooser.addOption("1", 1);
        reefSideChooser.addOption("2", 2);
        reefSideChooser.addOption("3", 3);
        reefSideChooser.addOption("4", 4);
        reefSideChooser.addOption("5", 5);
        reefSideChooser.addOption("6", 6);

        positionChooser.addOption("Left", 0);
        positionChooser.addOption("Middle", 1);
        positionChooser.addOption("Right", 2);

        heightChooser.addOption("L1", 1);
        heightChooser.addOption("L2", 2);
        heightChooser.addOption("L3", 3);

        Shuffleboard.getTab("AutomationSelectors").add(reefSideChooser);
        Shuffleboard.getTab("AutomationSelectors").add(positionChooser);
        Shuffleboard.getTab("AutomationSelectors").add(heightChooser);

    }

    public int getReefSide() {
        return reefSideChooser.getSelected() == null ? 1 : reefSideChooser.getSelected();
    }

    public int getPosition() {
        return positionChooser.getSelected() == null ? 1 : positionChooser.getSelected();
    }

    public int getHeight() {
        return heightChooser.getSelected() == null ? 1 : heightChooser.getSelected();
    }
}
