// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.EpilogueConfiguration;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.RobotSystemsCheckCommand;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.commands.elevator.MoveElevator;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.automation.AutomationSelector;
import frc.robot.automation.AutomatedScoring;

public class RobotContainer {

    public final DriveSubsystem driveSubsystem = new DriveSubsystem();
    public final VisionSubsystem visionSubsystem = new VisionSubsystem();
    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    private final Joystick driveJoystick = new Joystick(RobotConstants.PortConstants.Controller.DRIVE_JOYSTICK);
    private final Joystick operatorJoystick = new Joystick(RobotConstants.PortConstants.Controller.OPERATOR_JOYSTICK);

    public final AutomationSelector automationSelector = new AutomationSelector();

    SendableChooser<Command> m_autoPositionChooser = new SendableChooser<>();

    PowerDistribution pdp;

    private final Field2d field = new Field2d();

    public RobotContainer() {
        driveSubsystem.setDefaultCommand(new TeleopDriveCommand(driveSubsystem, driveJoystick));
        elevatorSubsystem.setDefaultCommand(new MoveElevator(elevatorSubsystem, operatorJoystick));
        createNamedCommands();

        configureButtonBindings();

        try {
            pdp = new PowerDistribution(16, ModuleType.kRev);
            m_autoPositionChooser = AutoBuilder.buildAutoChooser("Test Auto");
            Shuffleboard.getTab("Autonomous Selection").add(m_autoPositionChooser);
            Shuffleboard.getTab("Power").add(pdp);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void createNamedCommands() {
        // Add commands here to be able to execute in auto through pathplanner

        NamedCommands.registerCommand("Example", new RunCommand(() -> {
            System.out.println("Running...");
        }));
    }

    private void configureButtonBindings() {
        new JoystickButton(driveJoystick, 3).whileTrue(driveSubsystem.xCommand()); // Needs to be while true so the
                                                                                   // command ends
        new JoystickButton(driveJoystick, 4).whileTrue(driveSubsystem.gyroReset());

        // new JoystickButton(driveJoystick, 2).whileTrue(
        //         new SequentialCommandGroup(
        //                 new InstantCommand(() -> AutomatedScoring.Score(
        //                         automationSelector::getReefSide,
        //                         automationSelector::getPosition,
        //                         automationSelector::getHeight,
        //                         driveSubsystem, elevatorSubsystem).schedule())));

        

        // Above = DriveJoystick, Below = OperatorJoystick
    }

    public Command getAutonomousCommand() {
        if (m_autoPositionChooser.getSelected() != null) {
            return m_autoPositionChooser.getSelected();
        } else {
            return driveSubsystem.gyroReset();
        }
    }

    public Command getTestingCommand() {
        return new RobotSystemsCheckCommand(driveSubsystem);
    }

    public Field2d getField() {
        return field;
    }

    public final class UserPolicy {
        public static boolean xLocked = false;
        public static boolean isManualControlled = true;
    }
}
