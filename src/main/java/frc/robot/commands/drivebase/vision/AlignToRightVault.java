package frc.robot.commands.drivebase.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AlignToRightVault extends Command {

  private double angle, turn;

  public AlignToRightVault() {
    addRequirements(RobotContainer.drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angle = RobotContainer.drivebase.getDistanceToRightVault();
    turn = -angle / 180.0;

  /* if (Math.abs(angle) <= Constants.ALIGN_THRESHOLD) {
      turn = RobotContainer.getDriverRightX();
    } */

  /*  RobotContainer.swerveDrive.drive(
        RobotContainer.getDriverLeftY(), RobotContainer.getDriverLeftX(), turn, true, true); */
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}