// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 *
 * build.gradle file in the
 * project.
 */
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
            setUseTiming(true); // Use normal timing in sim

            // Optional: comment out the replay setup if not using AdvantageScope replay
            // String logPath = LogFileUtil.findReplayLog();
            // Logger.setReplaySource(new WPILOGReader(logPath));
            // Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath,
            // "_sim")));
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

            // Instead, just create a new log file like we do on real robot
            Logger.addDataReceiver(new WPILOGWriter("logs/sim-" + System.currentTimeMillis() + ".wpilog"));
        } else {
            System.out.println("Unknown robot mode. Please verify robot configuration.");
        }

        // Start logging
        Logger.start();
    }

    @Override
    public void robotInit() {
        // if (Robot.isReal()) {
        // CowboyUtils.RobotModes.currentMode = CowboyUtils.RobotModes.Mode.REAL;
        // } else if (Robot.isSimulation()) {
        // CowboyUtils.RobotModes.currentMode = CowboyUtils.RobotModes.Mode.SIM;
        // } else {
        // CowboyUtils.RobotModes.currentMode = CowboyUtils.RobotModes.Mode.REPLAY;
        // }
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     **/
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        // As stated
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
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
        // As stated
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        CommandScheduler.getInstance().cancelAll();
        // m_robotContainer.elevatorSubsystem.setMaxSpeeds(9000, 5000);
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