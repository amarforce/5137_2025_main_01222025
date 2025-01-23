package frc.robot.commands.coral;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class CoralPositioningCommand extends Command {
    private final Swerve swerveSubsystem;
    private final NetworkTable coralTable;
    
    // Constants for positioning
    private static final double DESIRED_DISTANCE = 0.5; // meters from L1
    private static final double YAW_TOLERANCE = 2.0; // degrees
    private static final double POSITION_TOLERANCE = 0.05; // meters

    public CoralPositioningCommand(Swerve swerve) {
        this.swerveSubsystem = swerve;
        this.coralTable = NetworkTableInstance.getDefault().getTable("Coral");
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Coral Position Status", "Aligning");
    }

    @Override
    public void execute() {
        // Get target information from NetworkTables
        double targetYaw = coralTable.getEntry("targetYaw").getDouble(0.0);
        
        // Calculate drive commands based on vision feedback
        double rotationSpeed = calculateRotationSpeed(targetYaw);
        double forwardSpeed = calculateForwardSpeed();
        
        // Apply drive commands
        swerveSubsystem.drive(
            forwardSpeed,  // forward
            0.0,          // strafe
            rotationSpeed,// rotation
            true         // fieldRelative
        );
        
        // Update status
        SmartDashboard.putNumber("Target Alignment Error", targetYaw);
    }

    private double calculateRotationSpeed(double targetYaw) {
        // Simple proportional control for rotation
        double kP = 0.03;
        return -targetYaw * kP;
    }

    private double calculateForwardSpeed() {
        // Implement distance-based approach speed
        // This would need to be calibrated based on your vision system's capabilities
        double kP = 0.5;
        double currentDistance = estimateDistance();
        return (currentDistance - DESIRED_DISTANCE) * kP;
    }

    private double estimateDistance() {
        // This would need to be implemented based on your vision system's capabilities
        // Could use target area or other vision measurements
        return 1.0; // Placeholder
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stop();
        SmartDashboard.putString("Coral Position Status", 
            interrupted ? "Interrupted" : "Completed");
    }

    @Override
    public boolean isFinished() {
        // Check if we're within tolerance of desired position
        double targetYaw = coralTable.getEntry("targetYaw").getDouble(0.0);
        return Math.abs(targetYaw) < YAW_TOLERANCE;
    }
}
