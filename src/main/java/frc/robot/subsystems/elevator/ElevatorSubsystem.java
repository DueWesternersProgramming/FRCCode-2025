package frc.robot.subsystems.elevator;

import edu.wpi.first.epilogue.EpilogueConfiguration;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    Mechanism2d elevatorMechanism2d;

    public ElevatorSubsystem() {
        if (RobotBase.isSimulation()){
            
        }
        // Add your constructor code here
    }

    public void goToScoreHeight() {
        // Add code here to move the elevator to the scoring height
    }

    @Override
    public void periodic() {
        
    }
    
}
