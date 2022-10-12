// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.RobotContainer.romiDrivetrain;

public class DriveToDistance extends CommandBase {
  private PIDController controller;
  private volatile boolean enabled = false;
  private double inches;
  /** Creates a new DefaultAutonomous. */
  public DriveToDistance(double inches) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(romiDrivetrain);
    this.inches = inches;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    romiDrivetrain.resetEncoders();
    controller = new PIDController(0.2, 0, 0);
    controller.setSetpoint(inches);
    controller.reset();
    enabled = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    romiDrivetrain.arcadeDrive(controller.calculate(romiDrivetrain.getLeftDistanceInch()) * 0.3, 0);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
