package frc.robot.commands.coral;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CoralVisionCommand extends Command {
    private final Vision visionSubsystem;
    private final NetworkTable coralTable;
    private PhotonTrackedTarget currentTarget;

    public CoralVisionCommand(Vision vision) {
        this.visionSubsystem = vision;
        this.coralTable = NetworkTableInstance.getDefault().getTable("Coral");
        addRequirements(vision);
    }

    @Override
    public void initialize() {
        coralTable.getEntry("tracking").setBoolean(false);
        SmartDashboard.putString("Coral Vision Status", "Searching...");
    }



    @Override
    public void execute() {
        currentTarget = visionSubsystem.getClosestCoral();
        
        if (currentTarget != null) {
            double yaw = currentTarget.getYaw();
            double pitch = currentTarget.getPitch();
            double area = currentTarget.getArea();
            
            coralTable.getEntry("tracking").setBoolean(true);
            coralTable.getEntry("targetYaw").setDouble(yaw);
            coralTable.getEntry("targetPitch").setDouble(pitch);
            coralTable.getEntry("targetArea").setDouble(area);
            
            SmartDashboard.putString("Coral Vision Status", "Target Acquired");
            SmartDashboard.putNumber("Coral Yaw", yaw);
        } else {
            coralTable.getEntry("tracking").setBoolean(false);
            SmartDashboard.putString("Coral Vision Status", "No Target");
        }
    }


    @Override
    public void end(boolean interrupted) {
        coralTable.getEntry("tracking").setBoolean(false);
        SmartDashboard.putString("Coral Vision Status", "Ended");
    }

    @Override
    public boolean isFinished() {
        return currentTarget != null;
    }
 
}
