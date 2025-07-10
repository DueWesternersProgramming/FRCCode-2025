// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.RobotSystemsCheckCommand;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.commands.elevator.MoveElevatorManual;
import frc.robot.commands.wrist.MoveWristManual;
import frc.robot.subsystems.claw.ClawIOSim;
import frc.robot.subsystems.claw.ClawIOSpark;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.claw.ClawSubsystemIO;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIONAVX;
import frc.robot.subsystems.drive.gyro.GyroIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystemIO;
import frc.robot.subsystems.elevator.ElevatorSubsystemIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystemIOSpark;
import frc.robot.subsystems.questnav.QuestNavIO;
import frc.robot.subsystems.questnav.QuestNavIOReal;
import frc.robot.subsystems.questnav.QuestNavSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.wrist.WristSubsystemIO;
import frc.robot.subsystems.wrist.WristSubsystemIOSim;
import frc.robot.subsystems.wrist.WristSubsystemIOSpark;
import frc.robot.automation.AutomationSelector;
import frc.robot.RobotConstants.PortConstants;
import frc.robot.utils.CowboyUtils;
import frc.robot.utils.CowboyUtils.RobotModes;
import frc.robot.RobotConstants.PortConstants.CAN;
import frc.robot.RobotConstants.ScoringConstants.Setpoints;
import frc.robot.automation.AutomatedScoring;

//@Logged(name = "RobotContainer")
public class RobotContainer {
        public final VisionSubsystem visionSubsystem = new VisionSubsystem();
        public final QuestNavSubsystem questNavSubsystem;
        public final DriveSubsystem driveSubsystem;
        public final ElevatorSubsystem elevatorSubsystem;
        public final WristSubsystem wristSubsystem;
        public final ClawSubsystem clawSubsystem;
        private final Joystick driveJoystick = new Joystick(RobotConstants.PortConstants.Controller.DRIVE_JOYSTICK);
        private final Joystick operatorJoystick = new Joystick(
                        RobotConstants.PortConstants.Controller.OPERATOR_JOYSTICK);

        ModuleIO[] moduleIOs;
        public final AutomationSelector automationSelector = new AutomationSelector();

        SendableChooser<Command> m_autoPositionChooser = new SendableChooser<>();

        PowerDistribution pdp;

        private final Field2d field = new Field2d();

        public RobotContainer() {
                System.out.println("Robot Mode: " + CowboyUtils.RobotModes.currentMode);

                switch (RobotModes.currentMode) {
                        case REAL:
                                // Real robot, instantiate hardware IO implementations

                                moduleIOs = new ModuleIO[] {
                                                new ModuleIOSpark(RobotConstants.PortConstants.CAN.FRONT_LEFT_DRIVING,
                                                                RobotConstants.PortConstants.CAN.FRONT_LEFT_TURNING,
                                                                RobotConstants.PortConstants.CAN.FRONT_LEFT_STEERING,
                                                                false),
                                                new ModuleIOSpark(RobotConstants.PortConstants.CAN.FRONT_RIGHT_DRIVING,
                                                                RobotConstants.PortConstants.CAN.FRONT_RIGHT_TURNING,
                                                                RobotConstants.PortConstants.CAN.FRONT_RIGHT_STEERING,
                                                                false),
                                                new ModuleIOSpark(RobotConstants.PortConstants.CAN.REAR_LEFT_DRIVING,
                                                                RobotConstants.PortConstants.CAN.REAR_LEFT_TURNING,
                                                                RobotConstants.PortConstants.CAN.REAR_LEFT_STEERING,
                                                                false),
                                                new ModuleIOSpark(RobotConstants.PortConstants.CAN.REAR_RIGHT_DRIVING,
                                                                RobotConstants.PortConstants.CAN.REAR_RIGHT_TURNING,
                                                                RobotConstants.PortConstants.CAN.REAR_RIGHT_STEERING,
                                                                false),
                                };
                                driveSubsystem = new DriveSubsystem(moduleIOs, new GyroIONAVX());

                                elevatorSubsystem = new ElevatorSubsystem(new ElevatorSubsystemIOSpark());

                                wristSubsystem = new WristSubsystem(new WristSubsystemIOSpark());

                                clawSubsystem = new ClawSubsystem(new ClawIOSpark());

                                questNavSubsystem = new QuestNavSubsystem(new QuestNavIOReal());

                                break;

                        case SIM:

                                moduleIOs = new ModuleIO[] {
                                                new ModuleIOSim(),
                                                new ModuleIOSim(),
                                                new ModuleIOSim(),
                                                new ModuleIOSim(),
                                };

                                driveSubsystem = new DriveSubsystem(moduleIOs, new GyroIOSim());

                                elevatorSubsystem = new ElevatorSubsystem(new ElevatorSubsystemIOSim());

                                wristSubsystem = new WristSubsystem(new WristSubsystemIOSim());

                                clawSubsystem = new ClawSubsystem(new ClawIOSim());

                                questNavSubsystem = new QuestNavSubsystem(new QuestNavIOReal());

                                break;

                        default:
                                // Replayed robot, disable IO implementations

                                moduleIOs = new ModuleIO[] {
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                };
                                driveSubsystem = new DriveSubsystem(moduleIOs, new GyroIO() {
                                });

                                elevatorSubsystem = new ElevatorSubsystem(new ElevatorSubsystemIO() {
                                });

                                wristSubsystem = new WristSubsystem(new WristSubsystemIO() {
                                });

                                clawSubsystem = new ClawSubsystem(new ClawSubsystemIO() {
                                });

                                questNavSubsystem = new QuestNavSubsystem(new QuestNavIO() {
                                });

                                break;
                }

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
                                AutomatedScoring.scoreCoralNoPathing(Setpoints.L1, elevatorSubsystem, wristSubsystem,
                                                clawSubsystem));
                NamedCommands.registerCommand("Score L2",
                                AutomatedScoring.scoreCoralNoPathing(Setpoints.L2, elevatorSubsystem, wristSubsystem,
                                                clawSubsystem));
                NamedCommands.registerCommand("Score L3",
                                AutomatedScoring.scoreCoralNoPathing(Setpoints.L3, elevatorSubsystem, wristSubsystem,
                                                clawSubsystem));

                NamedCommands.registerCommand("HumanPlayer",
                                AutomatedScoring.humanPlayerPickupNoPathing(driveSubsystem, elevatorSubsystem,
                                                wristSubsystem, clawSubsystem));

                NamedCommands.registerCommand("GrabLowAlgae",
                                AutomatedScoring.grabAlgaeNoPathing(Setpoints.L2, elevatorSubsystem, wristSubsystem,
                                                clawSubsystem));

                NamedCommands.registerCommand("GrabHighAlgae",
                                AutomatedScoring.grabAlgaeNoPathing(Setpoints.L3, elevatorSubsystem, wristSubsystem,
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

                new JoystickButton(driveJoystick, 6).onTrue(questNavSubsystem.calibrateCommand(driveSubsystem));

                new JoystickButton(driveJoystick, 1).onTrue(RobotState.setCanRotate(true))
                                .onFalse(RobotState.setCanRotate(false));

                new JoystickButton(driveJoystick, 3).onChange(driveSubsystem.xCommand()); // Needs to be while true so
                                                                                          // the
                                                                                          // command ends
                new JoystickButton(driveJoystick, 4).whileTrue(new SequentialCommandGroup(driveSubsystem.gyroReset(),
                                new InstantCommand(() -> questNavSubsystem.setRobotPose(driveSubsystem.getPose()),
                                                questNavSubsystem)));

                new JoystickButton(driveJoystick, 2).whileTrue(
                                Commands.deferredProxy(() -> AutomatedScoring.fullReefAutomation(
                                                automationSelector.getReefSide(),
                                                automationSelector.getPosition(),
                                                automationSelector.getHeight(),
                                                () -> -driveJoystick.getRawAxis(
                                                                PortConstants.Controller.DRIVE_COMMAND_Y_AXIS),
                                                driveSubsystem,
                                                elevatorSubsystem,
                                                wristSubsystem)));

                new JoystickButton(driveJoystick, 11).whileTrue(
                                AutomatedScoring.humanPlayerPickupNoPathing(
                                                driveSubsystem,
                                                elevatorSubsystem,
                                                wristSubsystem,
                                                clawSubsystem))
                                .onFalse(new SequentialCommandGroup(clawSubsystem.stopClaw(), new WaitCommand(1),
                                                AutomatedScoring.homeSubsystems(elevatorSubsystem, wristSubsystem)));

                new JoystickButton(driveJoystick, 9).whileTrue(clawSubsystem.intakeCoral())
                                .onFalse(clawSubsystem.stopClaw());
                new JoystickButton(driveJoystick, 7)
                                .onTrue(new SequentialCommandGroup(new WaitCommand(0.1), clawSubsystem.outtakeCoral()))
                                .onFalse(clawSubsystem.stopClaw());

                // Above = DriveJoystick, Below = OperatorJoystick

                // Manual claw controls. Triggers.
                new Trigger(() -> operatorJoystick.getRawAxis(2) > .2).whileTrue(clawSubsystem.intakeCoral())
                                .onFalse(clawSubsystem.stopClaw());
                new Trigger(() -> operatorJoystick.getRawAxis(3) > .2).whileTrue(clawSubsystem.outtakeCoral())
                                .onFalse(clawSubsystem.stopClaw());

                // Algae bottom (L2 algae), A button
                new JoystickButton(operatorJoystick, 1)
                                .whileTrue(AutomatedScoring.grabAlgaeNoPathing(Setpoints.L2, elevatorSubsystem,
                                                wristSubsystem,
                                                clawSubsystem));
                // .onFalse(AutomatedScoring.homeSubsystems(elevatorSubsystem, wristSubsystem));
                // Algae top (L3 algae), Y button
                new JoystickButton(operatorJoystick, 4)
                                .whileTrue(AutomatedScoring.grabAlgaeNoPathing(Setpoints.L3, elevatorSubsystem,
                                                wristSubsystem,
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
                                .whileTrue(AutomatedScoring.scoreCoralNoPathing(Setpoints.L1, elevatorSubsystem,
                                                wristSubsystem,
                                                clawSubsystem));

                // L2, RIGHT POV BUTTON
                new POVButton(operatorJoystick, 90)
                                .whileTrue(AutomatedScoring.scoreCoralNoPathing(Setpoints.L2, elevatorSubsystem,
                                                wristSubsystem,
                                                clawSubsystem));

                // L3, RIGHT POV BUTTON
                new POVButton(operatorJoystick, 0)
                                .whileTrue(AutomatedScoring.scoreCoralNoPathing(Setpoints.L3, elevatorSubsystem,
                                                wristSubsystem,
                                                clawSubsystem));

                new JoystickButton(operatorJoystick, 7)
                                .whileTrue(new MoveElevatorManual(elevatorSubsystem, operatorJoystick));
                new JoystickButton(operatorJoystick, 7)
                                .whileTrue(new MoveWristManual(wristSubsystem, operatorJoystick));

                new JoystickButton(operatorJoystick, 8).onTrue(new InstantCommand(() -> {
                        elevatorSubsystem.setEncoderValue(0);
                        wristSubsystem.setEncoderValue(0);

                }));

                new Trigger(() -> SmartDashboard.getBoolean("HomeSubsystems", false))
                                .onTrue(AutomatedScoring.homeSubsystems(elevatorSubsystem, wristSubsystem))
                                .onTrue(new InstantCommand(() -> SmartDashboard.putBoolean("HomeSubsystems", false)));

                new Trigger(() -> SmartDashboard.getBoolean("IntakeOn", false))
                                .whileTrue(clawSubsystem.intakeCoral())
                                .onFalse(clawSubsystem.stopClaw());
                new Trigger(() -> SmartDashboard.getBoolean("OuttakeOn", false))
                                .whileTrue(clawSubsystem.outtakeCoral())
                                .onFalse(clawSubsystem.stopClaw());

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
