package frc.robot.commands.coral;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.constants.IntakeConstants;

public class CoralScoreL1Command extends Command {
    private final Arm armSubsystem;
    private final Intake intakeSubsystem;
    
    private enum ScoreState {
        MOVING_TO_POSITION,
        RELEASING,
        RETRACTING
    }
    
    private ScoreState currentState;
    private double startTime;

    public CoralScoreL1Command(Arm arm, Intake intake) {
        this.armSubsystem = arm;
        this.intakeSubsystem = intake;
        addRequirements(arm, intake);
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
                armSubsystem.setGoal(0.0); // Set appropriate L1 position
                if (Math.abs(armSubsystem.getPose() - armSubsystem.getGoal()) < 0.1) {
                    currentState = ScoreState.RELEASING;
                    startTime = System.currentTimeMillis();
                    SmartDashboard.putString("Coral Score Status", "Releasing");
                }
                break;
                
            case RELEASING:
                intakeSubsystem.setSpeed(-IntakeConstants.defaultMotorSpeedIntake);
                if (System.currentTimeMillis() - startTime > 500) {
                    currentState = ScoreState.RETRACTING;
                    startTime = System.currentTimeMillis();
                    SmartDashboard.putString("Coral Score Status", "Retracting");
                }
                break;
                
            case RETRACTING:
                armSubsystem.setGoal(0.0); // Set stow position
                intakeSubsystem.stop();
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
        armSubsystem.setGoal(0.0); // Set safe position
        SmartDashboard.putString("Coral Score Status", 
            interrupted ? "Interrupted" : "Completed");
    }

    @Override
    public boolean isFinished() {
        return currentState == ScoreState.RETRACTING && 
               System.currentTimeMillis() - startTime > 1000;
    }
}
