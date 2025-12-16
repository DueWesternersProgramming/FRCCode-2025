package frc.robot.configurableAutos;

import java.util.ArrayList;
import java.util.Optional;

import frc.robot.RobotConstants.ConfigurableAutonsConstants.HumanPlayerPositions;
import frc.robot.RobotConstants.ConfigurableAutonsConstants.SidePosition;
import frc.robot.RobotConstants.ConfigurableAutonsConstants.StartingPositions;

public class ConfigurableAutons {

    public enum AutonStepType {
        INTAKE,
        SCORE,
        CLEAR_ALAGE,
    }

    public class AutonStep {
        public String actionDescription;
        public AutonStepType type;

        public AutonStep(String actionDescription, AutonStepType type, Optional<Object> HumanPlayerPosition,
                Optional<Object> StartingPosition, Optional<Object> SidePosition) {
            this.actionDescription = actionDescription;
            this.type = type;
            if (type == AutonStepType.INTAKE) {
                // Additional properties for INTAKE can be added here
            } else if (type == AutonStepType.SCORE) {
                // Additional properties for SCORE can be added here
            } else if (type == AutonStepType.CLEAR_ALAGE) {
                // Additional properties for CLEAR_ALAGE can be added here
            }
        }
    }

    public void GenerateAuton() {
        ArrayList<AutonStep> autonSteps = new ArrayList<>();

        // Example of adding steps to the autonomous routine
        autonSteps.add(new AutonStep("Intake from left side", AutonStepType.INTAKE,
                Optional.of(HumanPlayerPositions.LEFT), Optional.empty(), Optional.of(SidePosition.LEFT)));

        autonSteps.add(new AutonStep("Score at middle position", AutonStepType.SCORE,
                Optional.empty(), Optional.of(StartingPositions.MIDDLE), Optional.empty()));

        autonSteps.add(new AutonStep("Clear algae from right side", AutonStepType.CLEAR_ALAGE,
                Optional.empty(), Optional.empty(), Optional.of(SidePosition.ALGAE)));

        // Further processing of autonSteps can be done here

        for (AutonStep step : autonSteps) {
            System.out.println("Running now: " + step.actionDescription + ", Type: " + step.type);
        }
    }
}