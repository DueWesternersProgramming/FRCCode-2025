// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotConstants.SimMode;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer = new RobotContainer();

    public Robot() {
        Logger.recordMetadata("FRCCode-2025", "FRCCode-2025"); // Set a metadata value
        if (isReal()) {
            // In real robot mode, log to a USB stick and NetworkTables
            Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        } else if (isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
            System.out.println("Running in simulation mode: " + SimMode.SIM_MODE);

            if (SimMode.SIM_MODE == SimMode.SimModes.REPLAY) {
                setUseTiming(false); // Run as fast as possible in replay
                // Optional: comment out the replay setup if not using AdvantageScope replay
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                // Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath,
                // "_sim")));
            } else {
                setUseTiming(true); // Use normal timing in regular sim
            }

            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

            Logger.addDataReceiver(new WPILOGWriter("logs/sim-" + System.currentTimeMillis() + ".wpilog"));
        } else {
            System.out.println("Unknown robot mode. Please verify robot configuration.");
        }

        // Start logging
        Logger.start();
    }

    @Override
    public void robotInit() {
        Pathfinding.setPathfinder(new LocalADStarAK());
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     **/
    @Override
    public void robotPeriodic() {

        CommandScheduler.getInstance().run();
        // RobotState.visionPoseStatePeriodic(m_robotContainer.visionSubsystem,
        // m_robotContainer.questNavSubsystem);

        // if (DriverStation.isEnabled() &&
        // m_robotContainer.questNavSubsystem.isConnected()) {
        // RobotState.visionMode = VisionMode.QUEST_NAV_ONLY;
        // } else {
        // RobotState.visionMode = VisionMode.APRIL_TAG_ONLY;
        // }

    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_robotContainer.questNavSubsystem.setRobotPose(RobotState.robotPose);
        RobotState.isQuestNavPoseReset = true;

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();

        if (!DriverStation.isFMSAttached()) {
            m_robotContainer.questNavSubsystem.setRobotPose(RobotState.robotPose);
            RobotState.isQuestNavPoseReset = true;

        }

        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void simulationPeriodic() {

    }

    @Override
    public void simulationInit() {

    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();

        m_robotContainer.getTestingCommand().schedule();
    }

    @Override
    public void testPeriodic() {
    }
}