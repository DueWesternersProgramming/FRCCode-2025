package frc.robot.configurableAutos;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;

@FunctionalInterface
public interface AutoCommandFactory {
    Command create(Map<String, Integer> params);
}
