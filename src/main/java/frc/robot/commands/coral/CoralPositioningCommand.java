package frc.robot.commands.coral;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.constants.SwerveConstants;

public class CoralPositioningCommand extends Command {
    private final Swerve swerveSubsystem;
    private final NetworkTable coralTable;
    
    // Constants for positioning
    //private static final double DESIRED_DISTANCE = 0.5; // meters from L1
    private static final double YAW_TOLERANCE = 2.0; // degrees
    //private static final double POSITION_TOLERANCE = 0.05; // meters

    public CoralPositioningCommand(Swerve swerve) {
        this.swerveSubsystem = swerve;
        this.coralTable = NetworkTableInstance.getDefault().getTable("Coral");
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Coral Position Status", "Moving to L1");
    }

    @Override
    public void execute() {
        // Get current robot pose
        Pose2d currentPose = swerveSubsystem.getPose();
        double targetYaw = coralTable.getEntry("targetYaw").getDouble(0.0);
        
        // Calculate target pose for L1
        Pose2d targetPose = new Pose2d(
            currentPose.getTranslation(),
            Rotation2d.fromDegrees(currentPose.getRotation().getDegrees() - targetYaw)
        );
        
        // Use PathPlanner to move to position
        swerveSubsystem.driveToPose(targetPose);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(coralTable.getEntry("targetYaw").getDouble(0.0)) < YAW_TOLERANCE;
    }

}
