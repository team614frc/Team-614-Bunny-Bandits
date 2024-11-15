package frc.robot.commands.manipulator.helpergroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ManipulatorSubsystem;

public class Outtake extends Command {
  private double releaseSpeed;
  private final ManipulatorSubsystem manipulatorsubsystem;
  private boolean test = false;

  public Outtake(ManipulatorSubsystem manipulatorsubsystem, double releaseSpeed) {
    this.manipulatorsubsystem = manipulatorsubsystem;
    addRequirements(manipulatorsubsystem);
    test = true;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    manipulatorsubsystem.enable();
    if (test) {
      releaseSpeed = SmartDashboard.getNumber("Shooter Test", 1900);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    manipulatorsubsystem.setSetpoint(releaseSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return manipulatorsubsystem.atGoal(releaseSpeed);
  }
   
}
