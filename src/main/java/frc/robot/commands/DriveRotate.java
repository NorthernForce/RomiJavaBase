// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.RobotContainer.romiDrivetrain;

public class DriveRotate extends CommandBase {
  private PIDController leftController, rightController;
  private final double radians;
  private final static double romiRadius = 2.93;
  /** Creates a new DefaultAutonomous. */
  public DriveRotate(double radians) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(romiDrivetrain);
    this.radians = radians;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    romiDrivetrain.resetEncoders();
    leftController = new PIDController(0.15, 0, 0);
    rightController = new PIDController(0.15, 0, 0);
    leftController.setSetpoint(radians * romiRadius);
    leftController.setIntegratorRange(-0.5, 0.5);
    rightController.setIntegratorRange(-0.5, 0.5);
    leftController.setTolerance(0.05, 0.5);
    rightController.setSetpoint(-radians * romiRadius);
    System.out.println("Setpoint: " + radians * romiRadius);
    rightController.setTolerance(0.05, 0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    romiDrivetrain.tankDrive(
      leftController.calculate(romiDrivetrain.getLeftDistanceInch()), 
      rightController.calculate(romiDrivetrain.getRightDistanceInch())
    );
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rightController.atSetpoint() && leftController.atSetpoint();
  }
}
