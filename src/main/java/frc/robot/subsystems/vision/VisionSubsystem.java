package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotConstants.SubsystemEnabledConstants;
import frc.robot.RobotConstants.VisionConstants;
import frc.robot.RobotConstants.VisionConstants.AprilTagCameraConfig;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.CowboyUtils;
import frc.robot.utils.CowboyUtils.RobotModes;

import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.VisionSystemSim;

public class VisionSubsystem extends SubsystemBase {
    public static VisionSystemSim visionSim;

    public static record AprilTagCamera(
            AprilTagCameraIO io,
            AprilTagIOInputsAutoLogged inputs,
            VisionConstants.VisionSource source) {
    }

    public List<AprilTagCamera> aprilTagCameras = new ArrayList<>();

    public VisionSubsystem() {
        for (AprilTagCameraConfig config : VisionConstants.CAMERA_CONFIGS) {
            AprilTagCameraIO io;

            switch (RobotModes.currentMode) {
                case REAL:
                    if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {
                        io = new AprilTagCameraIOPhoton(
                                config.source());

                    } else {
                        io = new AprilTagCameraIO() {
                        };
                    }
                    break;
                case SIM:
                    if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {
                        io = new AprilTagCameraIOPhotonSim(
                                config.source(),
                                config.simConfig());
                    } else {
                        io = new AprilTagCameraIO() {
                        };
                    }
                    break;
                case REPLAY:
                default:
                    io = new AprilTagCameraIO() {
                    };
                    break;
            }

            aprilTagCameras.add(new AprilTagCamera(io, new AprilTagIOInputsAutoLogged(), config.source()));

        }
    }

    @Override
    public void periodic() {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {
            if (RobotBase.isSimulation()) {
                VisionConstants.aprilTagSim.get().update(RobotState.robotPose);
            }
            for (AprilTagCamera camera : aprilTagCameras) {
                camera.io.updateInputs(camera.inputs);
                Logger.processInputs("VisionSubsystem/" + camera.source.name(), camera.inputs);
            }
        }
    }
}