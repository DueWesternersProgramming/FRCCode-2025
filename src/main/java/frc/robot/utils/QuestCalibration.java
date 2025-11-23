package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.questnav.QuestNavSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class QuestCalibration {

        /**
         * Collects and outputs calibration data by rotating the robot and recording
         * uncorrected
         * poses from the QuestNav system.
         * 
         * @param driveOutput         A Consumer method that accepts ChassisSpeeds to
         *                            control
         *                            the robot's movement.
         * @param resetPose           A Consumer method that accepts a Pose2d to reset
         *                            the main
         *                            odometry everything is measured from.
         * @param getQuestUncorrected A Supplier method that provides the current
         *                            uncorrected
         *                            Pose2d from the QuestNav system.
         * @param driveSubsystem
         * @param questNavSubsystem
         * @return Peforms the calibration data
         *         collection routine. It resets the robot's pose, then rotates the
         *         robot slowly
         *         and collects uncorrected poses to feed into desmos for the circle
         *         equation regression. It will print the CSV formatted data
         *         to the console once complete.
         */
        public static Command CollectCalibrationDataCommand(
                        Consumer<ChassisSpeeds> driveOutput,
                        Consumer<Pose2d> resetPose,
                        Supplier<Pose2d> getQuestUncorrected,
                        DriveSubsystem driveSubsystem,
                        QuestNavSubsystem questNavSubsystem) {

                double turnDegreesPerSecondRate = 37.5;

                ArrayList<Pose2d> collectedPoses = new ArrayList<>();

                double rotationTime = 360 / turnDegreesPerSecondRate;
                double samplePeriod = rotationTime / 15.0; // 15 samples around circle

                Command turningCommand = Commands.run(
                                () -> driveOutput.accept(
                                                new ChassisSpeeds(
                                                                0,
                                                                0,
                                                                Units.degreesToRadians(turnDegreesPerSecondRate))))
                                .beforeStarting(() -> {
                                        System.out.println("Starting Quest Calibration Data Collection");
                                        resetPose.accept(new Pose2d());
                                })
                                .withTimeout(rotationTime);

                // Command that samples poses periodically
                Timer timer = new Timer();
                Command samplingCommand = Commands.run(() -> {
                        if (timer.advanceIfElapsed(samplePeriod)) {
                                collectedPoses.add(getQuestUncorrected.get());
                        }
                })
                                .beforeStarting(() -> timer.restart());

                // Run sampling while turning
                return new ParallelDeadlineGroup(
                                turningCommand,
                                samplingCommand).andThen(() -> {
                                        driveOutput.accept(new ChassisSpeeds());
                                        String csvText = ConvertCordsToCSVText(collectedPoses);
                                        System.out.println(csvText);
                                }, driveSubsystem, questNavSubsystem);
        }

        public static String ConvertCordsToCSVText(List<Pose2d> poses) {
                String csvText = "X, Y\n";
                for (Pose2d pose : poses) {
                        csvText += pose.getX() + ", " + pose.getY() + "\n";
                }
                // System.out.println(csvText);
                return csvText;

        }

}