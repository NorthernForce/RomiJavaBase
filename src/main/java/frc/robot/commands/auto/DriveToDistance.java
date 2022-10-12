// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.RobotContainer.romiDrivetrain;

public class DriveToDistance extends CommandBase {
  private final PIDController pid;

  /** Creates a new DriveToDistance. 
   * @param distance this should be the distance in inches you wish to go.
  */

  public DriveToDistance(double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(romiDrivetrain);
    pid = new PIDController(0.2, 0, 0);
    pid.setSetpoint(distance);
    pid.setTolerance(0.1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    romiDrivetrain.resetEncoders();
    pid.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    romiDrivetrain.arcadeDrive(pid.calculate((romiDrivetrain.getLeftDistanceInch()+romiDrivetrain.getRightDistanceInch())/2), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
