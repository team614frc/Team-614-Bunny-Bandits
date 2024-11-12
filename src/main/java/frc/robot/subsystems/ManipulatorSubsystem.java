// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ManipulatorConstants;

public class ManipulatorSubsystem extends PIDSubsystem {
  /** Creates a new ShooterSubsystem. */
  CANSparkFlex shooterMotor;


  public ManipulatorSubsystem() {
    super(
        // The PIDController used by the subsystem
        new PIDController(Constants.ManipulatorConstants.SHOOTER_kP, 0, 0));

    shooterMotor = new CANSparkFlex(Constants.ManipulatorConstants.SHOOTER_MOTOR, MotorType.kBrushless);
    // shooterMotor.restoreFactoryDefaults();
    shooterMotor.setSmartCurrentLimit(Constants.ManipulatorConstants.MOTOR_CURRENT_LIMIT);
    shooterMotor.setInverted(true);
    shooterMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    shooterMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    shooterMotor.setVoltage(-(output + getController().calculate(getMeasurement(), setpoint)));
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    SmartDashboard.putNumber("SHOOTER RPM", getManipulatorVelocity());
    return getManipulatorVelocity();
  }

  public double getManipulatorVelocity() {
    return shooterMotor.getEncoder().getVelocity();
  }

  public void set(double speed) {
    shooterMotor.set(-speed);
  }

  public boolean atGoal(double goal) {
    return Math.abs(getMeasurement() + goal) <= ManipulatorConstants.SHOOTER_THRESHOLD;
  }
}