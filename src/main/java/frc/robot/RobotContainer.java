package frc.robot;


import java.io.File;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.elastic.*;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.commands.coral.CoralPositioningCommand;
import frc.robot.commands.coral.CoralScoreL1Command;
import frc.robot.commands.coral.CoralVisionCommand;

@SuppressWarnings("unused")
public class RobotContainer {
	private CommandPS5Controller driver;
	private CommandPS5Controller operator;

	private Vision vision;
	private Swerve swerve;
	private Elevator elevator;
	private Arm arm;
	private Wrist wrist;
	private Intake intake;
    private Hang hang;

	private SwerveCommands swerveCommands;
	private ElevatorCommands elevatorCommands;
	private ArmCommands armCommands;
	private WristCommands wristCommands;
	private IntakeCommands intakeCommands;
    private HangCommand hangCommand;

	    // New coral command declarations
    private final CoralVisionCommand coralVisionCommand;
    private final CoralPositioningCommand coralPositionCommand;
    private final CoralScoreL1Command coralScoreCommand;

	private Reef reef;

	public RobotContainer() {
		driver = new CommandPS5Controller(0);
		operator = new CommandPS5Controller(1);

		vision = new Vision();
		swerve = new Swerve(new File(Filesystem.getDeployDirectory(),"swerve.json"), vision);
		elevator = new Elevator();
		arm = new Arm();
		wrist = new Wrist();
		intake = new Intake();
    	hang = new Hang();

		swerveCommands = new SwerveCommands(swerve);
		elevatorCommands = new ElevatorCommands(elevator);
		armCommands = new ArmCommands(arm);
		wristCommands = new WristCommands(wrist);
		intakeCommands = new IntakeCommands(intake);
    	hangCommand = new HangCommand(hang);

		// Initialize new coral commands
        coralVisionCommand = new CoralVisionCommand(vision);
        coralPositionCommand = new CoralPositioningCommand(swerve);
        coralScoreCommand = new CoralScoreL1Command(arm, wrist, intake);

		reef = new Reef();
		SmartDashboard.putData("Reef", reef);

		// Initialize Network Table for coral automation
        // This sets the "Status" entry in the "Coral" table to "Ready"
        // It indicates that the system is ready for coral automation tasks.
        NetworkTableInstance.getDefault().getTable("Coral").getEntry("Status").setString("Ready");
        // Ensure the statement is complete with a semicolon at the end.

		configureBindings();
	}

	private void configureBindings() {
		

		
		// Driver Bindings

		swerve.setDefaultCommand(
			swerveCommands.drive(
				() -> -driver.getLeftY(),
				() -> -driver.getLeftX(),
				() -> -driver.getRightX(),
				() -> driver.R1().negate().getAsBoolean())
		);

		// Re-add the driver's cross button binding for swerve lock
		driver.cross().whileTrue(swerveCommands.lock());

		driver.triangle().onTrue(swerveCommands.driveToStation());
		driver.square().onTrue(swerveCommands.driveToCage());
		driver.circle().onTrue(swerveCommands.driveToProcessor());

		driver.povLeft().onTrue(swerveCommands.driveToReefLeft());
		driver.povUp().onTrue(swerveCommands.driveToReefCenter());
		driver.povRight().onTrue(swerveCommands.driveToReefRight());

		driver.options().onTrue(swerveCommands.resetGyro());

		/*
		driver.povUp().onTrue(new InstantCommand(() -> swerve.setRoutine(swerve.m_sysIdRoutineTranslation)));
		driver.povLeft().onTrue(new InstantCommand(() -> swerve.setRoutine(swerve.m_sysIdRoutineSteer)));
		driver.povRight().onTrue(new InstantCommand(() -> swerve.setRoutine(swerve.m_sysIdRoutineRotation)));
		driver.options().and(driver.povDown().negate()).whileTrue(swerveCommands.sysIdDynamic(Direction.kForward));
		driver.options().and(driver.povDown()).whileTrue(swerveCommands.sysIdDynamic(Direction.kForward));
		driver.create().and(driver.povDown().negate()).whileTrue(swerveCommands.sysIdQuasistatic(Direction.kReverse));
		driver.create().and(driver.povDown()).whileTrue(swerveCommands.sysIdQuasistatic(Direction.kReverse));
		*/

		driver.touchpad().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

		        // Add new coral automation binding to driver's L1 button
		driver.L1().onTrue(
			Commands.sequence(
				// Start with a command to update dashboard
				new InstantCommand(() -> 
					SmartDashboard.putString("Automation Status", "Starting Coral L1 Sequence")),
						
				// Run vision tracking until target acquired
				coralVisionCommand.until(() -> 
					NetworkTableInstance.getDefault()
						.getTable("Coral")
						.getEntry("tracking")
						.getBoolean(false)),
						
				// Position robot for scoring
				coralPositionCommand,
						
				// Execute scoring sequence
				coralScoreCommand,
						
				// Clean up and reset status
				new InstantCommand(() -> {
					SmartDashboard.putString("Automation Status", "Sequence Complete");
					NetworkTableInstance.getDefault()
						.getTable("Coral")
						.getEntry("Status")
						.setString("Ready");
				})
			)
		);
		// Operator Bindings

		elevator.setDefaultCommand(elevatorCommands.setGoal(() -> 1 - operator.getLeftY()));
		arm.setDefaultCommand(armCommands.setSpeed(() -> operator.getRightX()));

		operator.triangle()
			.onTrue(elevatorCommands.moveToL4())
			.onTrue(armCommands.moveToL4());

		operator.circle()
			.onTrue(elevatorCommands.moveToL3())
			.onTrue(armCommands.moveToL3());

		operator.square()
			.onTrue(elevatorCommands.moveToL2())
			.onTrue(armCommands.moveToL2());

		operator.cross()
			.onTrue(elevatorCommands.moveToL1())
			.onTrue(armCommands.moveToL1());

		operator.R1()
			.onTrue(wristCommands.wristForward())
			.onFalse(wristCommands.wristReverse());

		operator.L2()
			.onTrue(intakeCommands.intakeReverse())
			.onFalse(intakeCommands.stop());

		operator.R2()
			.onTrue(intakeCommands.intakeForward())
			.onFalse(intakeCommands.stop());
    
    operator.touchpad()
      .onTrue(hangCommand);
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}