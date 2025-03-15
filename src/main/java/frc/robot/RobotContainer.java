// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.EpilogueConfiguration;
import edu.wpi.first.epilogue.Logged;
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
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.RobotSystemsCheckCommand;
import frc.robot.commands.claw.SetClawSpeed;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.commands.elevator.MoveElevatorManual;
import frc.robot.commands.wrist.MoveWristManual;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.automation.AutomationSelector;
import frc.robot.RobotConstants.PortConstants.CAN;
import frc.robot.automation.AutomatedScoring;

//@Logged(name = "RobotContainer")
public class RobotContainer {
        public final VisionSubsystem visionSubsystem = new VisionSubsystem();
        public final DriveSubsystem driveSubsystem = new DriveSubsystem();
        public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
        public final WristSubsystem wristSubsystem = new WristSubsystem();
        public final ClawSubsystem clawSubsystem = new ClawSubsystem();

        private final Joystick driveJoystick = new Joystick(RobotConstants.PortConstants.Controller.DRIVE_JOYSTICK);
        private final Joystick operatorJoystick = new Joystick(
                        RobotConstants.PortConstants.Controller.OPERATOR_JOYSTICK);

        public final AutomationSelector automationSelector = new AutomationSelector();

        SendableChooser<Command> m_autoPositionChooser = new SendableChooser<>();

        PowerDistribution pdp;

        private final Field2d field = new Field2d();

        public RobotContainer() {
                driveSubsystem.setDefaultCommand(new TeleopDriveCommand(driveSubsystem, driveJoystick));

                // elevatorSubsystem.setDefaultCommand(new MoveElevatorManual(elevatorSubsystem,
                // operatorJoystick));
                // wristSubsystem.setDefaultCommand(new MoveWristManual(wristSubsystem,
                // operatorJoystick));

                createNamedCommands();

                configureButtonBindings();

                try {
                        pdp = new PowerDistribution(CAN.PDH, ModuleType.kRev);
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
                NamedCommands.registerCommand("Score L1",
                                AutomatedScoring.scoreCoralNoPathing(1, elevatorSubsystem, wristSubsystem,
                                                clawSubsystem));
                NamedCommands.registerCommand("Score L2",
                                AutomatedScoring.scoreCoralNoPathing(2, elevatorSubsystem, wristSubsystem,
                                                clawSubsystem));
                NamedCommands.registerCommand("Score L3",
                                AutomatedScoring.scoreCoralNoPathing(3, elevatorSubsystem, wristSubsystem,
                                                clawSubsystem));

                NamedCommands.registerCommand("HumanPlayer",
                                AutomatedScoring.humanPlayerPickupNoPathing(driveSubsystem, elevatorSubsystem,
                                                wristSubsystem, clawSubsystem));

                NamedCommands.registerCommand("GrabLowAlgae",
                                AutomatedScoring.grabAlgaeNoPathing(2, elevatorSubsystem, wristSubsystem,
                                                clawSubsystem));

                NamedCommands.registerCommand("GrabHighAlgae",
                                AutomatedScoring.grabAlgaeNoPathing(3, elevatorSubsystem, wristSubsystem,
                                                clawSubsystem));

                NamedCommands.registerCommand("CoralIn", clawSubsystem.intakeCoral());
                NamedCommands.registerCommand("CoralOut", clawSubsystem.outtakeCoral());
                NamedCommands.registerCommand("AlgaeIn", clawSubsystem.intakeAlgae());
                NamedCommands.registerCommand("AlgaeOut", clawSubsystem.outtakeAlgae());

                NamedCommands.registerCommand("AlgaeIn", clawSubsystem.intakeAlgae());

                NamedCommands.registerCommand("StopClaw", clawSubsystem.stopClaw());

                NamedCommands.registerCommand("ProcessorHome",
                                AutomatedScoring.homeSubsystems(elevatorSubsystem, wristSubsystem));

        }

        private void configureButtonBindings() {
                new JoystickButton(driveJoystick, 1).onTrue(RobotState.setCanRotate(true))
                                .onFalse(RobotState.setCanRotate(false));

                new JoystickButton(driveJoystick, 3).onChange(driveSubsystem.xCommand()); // Needs to be while true so
                                                                                          // the
                                                                                          // command ends
                new JoystickButton(driveJoystick, 4).whileTrue(driveSubsystem.gyroReset());

                new JoystickButton(driveJoystick, 2).whileTrue(
                                new InstantCommand(() -> AutomatedScoring.fullReefAutomation(
                                                automationSelector.getReefSide(),
                                                automationSelector.getPosition(),
                                                automationSelector.getHeight(),
                                                driveSubsystem,
                                                elevatorSubsystem,
                                                wristSubsystem,
                                                clawSubsystem).schedule()))
                                .onFalse(new InstantCommand(() -> {
                                        driveSubsystem.drive(0, 0, 0, false, false);
                                }, driveSubsystem));

                new JoystickButton(driveJoystick, 11).whileTrue(
                                new InstantCommand(() -> AutomatedScoring.humanPlayerPickup(
                                                automationSelector.getHumanPlayerStation(),
                                                driveSubsystem,
                                                elevatorSubsystem,
                                                wristSubsystem,
                                                clawSubsystem).schedule()))
                                .onFalse(new InstantCommand(() -> {
                                        driveSubsystem.drive(0, 0, 0, false, false);
                                }, driveSubsystem));

                new JoystickButton(driveJoystick, 9).whileTrue(clawSubsystem.intakeCoral()).onFalse(clawSubsystem.stopClaw());
                new JoystickButton(driveJoystick, 10).whileTrue(clawSubsystem.outtakeCoral()).onFalse(clawSubsystem.stopClaw());

                // Above = DriveJoystick, Below = OperatorJoystick

                // Manual claw controls. Triggers. Left is always intake regardless of game
                // peice.
                new Trigger(() -> operatorJoystick.getRawAxis(2) > .2).whileTrue(clawSubsystem.intakeCoral())
                                .onFalse(clawSubsystem.stopClaw());
                new Trigger(() -> operatorJoystick.getRawAxis(3) > .2).whileTrue(clawSubsystem.outtakeCoral())
                                .onFalse(clawSubsystem.stopClaw());

                // Algae bottom (L2 algae), A button
                new JoystickButton(operatorJoystick, 1)
                                .whileTrue(AutomatedScoring.grabAlgaeNoPathing(2, elevatorSubsystem, wristSubsystem,
                                                clawSubsystem));
                // .onFalse(AutomatedScoring.homeSubsystems(elevatorSubsystem, wristSubsystem));
                // Algae top (L3 algae), Y button
                new JoystickButton(operatorJoystick, 4)
                                .whileTrue(AutomatedScoring.grabAlgaeNoPathing(3, elevatorSubsystem, wristSubsystem,
                                                clawSubsystem));
                // .onFalse(AutomatedScoring.homeSubsystems(elevatorSubsystem, wristSubsystem));

                new JoystickButton(operatorJoystick, 5)
                                .whileTrue(AutomatedScoring.homeSubsystems(elevatorSubsystem, wristSubsystem));

                // Human Player, LEFT POV BUTTON
                new POVButton(operatorJoystick, 270)
                                .whileTrue(AutomatedScoring.humanPlayerPickupNoPathing(driveSubsystem,
                                                elevatorSubsystem,
                                                wristSubsystem, clawSubsystem));

                // L1, DOWN POV BUTTON
                new POVButton(operatorJoystick, 180)
                                .whileTrue(AutomatedScoring.scoreCoralNoPathing(1, elevatorSubsystem, wristSubsystem,
                                                clawSubsystem));

                // L2, RIGHT POV BUTTON
                new POVButton(operatorJoystick, 90)
                                .whileTrue(AutomatedScoring.scoreCoralNoPathing(2, elevatorSubsystem, wristSubsystem,
                                                clawSubsystem));

                // L3, RIGHT POV BUTTON
                new POVButton(operatorJoystick, 0)
                                .whileTrue(AutomatedScoring.scoreCoralNoPathing(3, elevatorSubsystem, wristSubsystem,
                                                clawSubsystem));

                new JoystickButton(operatorJoystick, 7)
                                .whileTrue(new MoveElevatorManual(elevatorSubsystem, operatorJoystick));
                new JoystickButton(operatorJoystick, 7)
                                .whileTrue(new MoveWristManual(wristSubsystem, operatorJoystick));

                new JoystickButton(operatorJoystick, 8).onTrue(new InstantCommand(() -> {
                        elevatorSubsystem.setEncoderValue(0);
                        wristSubsystem.setEncoderValue(0);

                }));
                new JoystickButton(operatorJoystick, 6).whileTrue(clawSubsystem.yeetAlgae()).onFalse(clawSubsystem.stopClaw());
        }

        public Command getAutonomousCommand() {
                if (m_autoPositionChooser.getSelected() != null) {
                        return m_autoPositionChooser.getSelected();
                } else {
                        return driveSubsystem.gyroReset();
                }
        }

        public Command getTestingCommand() {
                return new RobotSystemsCheckCommand(driveSubsystem, elevatorSubsystem, wristSubsystem, clawSubsystem);
        }

        public Field2d getField() {
                return field;
        }

}
