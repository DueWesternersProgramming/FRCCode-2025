package frc.robot.configurableAutos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.ConfigurableAutonsConstants.HumanPlayerPositions;
import frc.robot.RobotConstants.ConfigurableAutonsConstants.SidePosition;
import frc.robot.RobotConstants.ConfigurableAutonsConstants.StartingPositions;

public class ConfigurableAutonsSelector {
    public ConfigurableAutonsSelector() {
        // Configure a chooser for all selection options

        SendableChooser<StartingPositions> startingPositionChooser = new SendableChooser<>();
        for (StartingPositions position : StartingPositions.values()) {
            startingPositionChooser.addOption(position.name(), position);
        }

    }
}
