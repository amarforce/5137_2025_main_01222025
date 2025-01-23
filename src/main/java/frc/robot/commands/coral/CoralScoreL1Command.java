package frc.robot.commands.coral;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Intake;

public class CoralScoreL1Command extends Command {
    private final Arm armSubsystem;
    private final Wrist wristSubsystem;
    private final Intake intakeSubsystem;
    
    private enum ScoreState {
        MOVING_TO_POSITION,
        RELEASING,
        RETRACTING
    }
    
    private ScoreState currentState;
    private double startTime;

    public CoralScoreL1Command(Arm arm, Wrist wrist, Intake intake) {
        this.armSubsystem = arm;
        this.wristSubsystem = wrist;
        this.intakeSubsystem = intake;
        addRequirements(arm, wrist, intake);
    }

    @Override
    public void initialize() {
        currentState = ScoreState.MOVING_TO_POSITION;
        startTime = System.currentTimeMillis();
        SmartDashboard.putString("Coral Score Status", "Moving to L1");
    }

    @Override
    public void execute() {
        switch (currentState) {
            case MOVING_TO_POSITION:
                // Move arm to L1 position
                armSubsystem.moveToL1Position();
                // Check if arm is in position
                if (armSubsystem.isAtTarget()) {
                    currentState = ScoreState.RELEASING;
                    startTime = System.currentTimeMillis();
                    SmartDashboard.putString("Coral Score Status", "Releasing");
                }
                break;
                
            case RELEASING:
                // Release the coral
                intakeSubsystem.setSpeed(-0.3); // Reverse intake to release
                if (System.currentTimeMillis() - startTime > 500) { // 0.5 second release
                    currentState = ScoreState.RETRACTING;
                    startTime = System.currentTimeMillis();
                    SmartDashboard.putString("Coral Score Status", "Retracting");
                }
                break;
                
            case RETRACTING:
                // Return to safe position
                armSubsystem.moveToStowPosition();
                intakeSubsystem.stop();
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
        SmartDashboard.putString("Coral Score Status", 
            interrupted ? "Interrupted" : "Completed");
    }

    @Override
    public boolean isFinished() {
        return currentState == ScoreState.RETRACTING && 
               System.currentTimeMillis() - startTime > 1000; // 1 second retract time
    }
}