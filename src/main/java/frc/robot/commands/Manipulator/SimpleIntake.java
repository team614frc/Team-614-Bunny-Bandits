package frc.robot.commands.Manipulator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotContainer;

public class SimpleIntake extends Command {

  public double intakeSpeed;

  /** Creates a new Intake. */
  public SimpleIntake(double intakeSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    SmartDashboard.putNumber("IntakeSpeed", intakeSpeed);
    addRequirements(RobotContainer.intakeSubsystem);
    intakeSpeed = IntakeConstants.INTAKE_SPEED;
    this.intakeSpeed = intakeSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString(getSubsystem(), getName());
    RobotContainer.intakeSubsystem.setIntake(intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intakeSubsystem.setIntake(IntakeConstants.INTAKE_REST_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
