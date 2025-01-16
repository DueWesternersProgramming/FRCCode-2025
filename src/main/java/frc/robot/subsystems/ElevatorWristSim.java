package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotConstants.ElevatorConstants;

public class ElevatorWristSim {

    private static MechanismLigament2d m_elevator;
    private static MechanismLigament2d m_wrist;
    static double ElevatorLength = ElevatorConstants.SIM_CONSTANTS.MIN_LIGMENT_LENGTH;
    static double WristAngle = 90;

    public ElevatorWristSim() {
        Mechanism2d mech = new Mechanism2d(3, 6);
        // the mechanism root node
        MechanismRoot2d root = mech.getRoot("climber", 1.5, 0);

        // MechanismLigament2d objects represent each "section"/"stage" of the
        // mechanism, and are based
        // off the root node or another ligament object
        m_elevator = root
                .append(new MechanismLigament2d("elevator", ElevatorConstants.SIM_CONSTANTS.MIN_LIGMENT_LENGTH, 85, 6,
                        new Color8Bit(Color.kRed)));

        m_wrist = m_elevator.append(
                new MechanismLigament2d("wrist", 0.5, 90, 6, new Color8Bit(Color.kBlue)));

        SmartDashboard.putData("Mech2d", mech);
    }

    public static void update() {
        m_elevator.setLength(ElevatorLength);
        // m_wrist.setAngle(WristAngle);
    }

    public static void setElevatorLength(double level) {
        if (level == 1) {
            ElevatorLength = ElevatorConstants.SIM_CONSTANTS.L1;
        } else if (level == 2) {
            ElevatorLength = ElevatorConstants.SIM_CONSTANTS.L2;
        } else if (level == 3) {
            ElevatorLength = ElevatorConstants.SIM_CONSTANTS.L3;
        } else {
            ElevatorLength = ElevatorConstants.SIM_CONSTANTS.L1;
        }

    }

    public static void setWristAngle(double angle) {
        WristAngle = angle;
    }
}
