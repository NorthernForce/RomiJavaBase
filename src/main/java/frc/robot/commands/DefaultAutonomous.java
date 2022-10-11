// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.RobotContainer.romiDrivetrain;

public class DefaultAutonomous extends CommandBase {
  PIDController controller;
  volatile boolean enabled = false;
  /** Creates a new DefaultAutonomous. */
  public DefaultAutonomous() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(romiDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    romiDrivetrain.arcadeDrive(controller.calculate(romiDrivetrain.getLeftDistanceInch()), 0);
  }
  public void driveForward(double inches)
  {
    romiDrivetrain.resetEncoders();
    controller = new PIDController(1, 0, 0);
    controller.setSetpoint(inches);
    enabled = true;
    while (enabled);
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
