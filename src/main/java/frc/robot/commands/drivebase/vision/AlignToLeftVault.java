package frc.robot.commands.drivebase.vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AlignToLeftVault extends Command {

  private double angle, turn;

  public AlignToLeftVault() {
    addRequirements(RobotContainer.drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angle = RobotContainer.drivebase.getDistanceToLeftVault();
    turn = -angle / 180.0;

    if (Math.abs(angle) <= Constants.VisionConstants.ALIGN_THRESHOLD) {
      turn = RobotContainer.driverXbox.getRightX();
    }
    Translation2d translation2d =
        new Translation2d(
            RobotContainer.driverXbox.getLeftY(), RobotContainer.driverXbox.getLeftX());
    RobotContainer.drivebase.drive(translation2d, turn, true);
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
