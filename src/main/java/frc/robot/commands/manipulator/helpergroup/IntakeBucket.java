package frc.robot.commands.manipulator.helpergroup;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.RobotContainer;

public class IntakeBucket extends Command {
      private double intakeSpeed;
      private ManipulatorSubsystem manipulatorSubsystem;

  /** Creates a new Intake. */
  public IntakeBucket(ManipulatorSubsystem manipulatorSubsystem, double intakeSpeed) {
    this.intakeSpeed = intakeSpeed;
    this.manipulatorSubsystem = manipulatorSubsystem;
    addRequirements(manipulatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    manipulatorSubsystem.set(intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    manipulatorSubsystem.set(ManipulatorConstants.INTAKE_REST_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
