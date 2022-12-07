// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.PIDBuilder;

import static frc.robot.RobotContainer.romiDrivetrain;

public class DriveToDistance extends CommandBase {
  private final PIDController leftPID;
  private final PIDController rightPID;

  /** Creates a new DriveToDistance. 
   * @param distance this should be the distance in inches you wish to go.
  */

  public DriveToDistance(double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(romiDrivetrain);
    PIDBuilder pidBuild = new PIDBuilder(0.2, 0, 0, 0.1, distance);
    leftPID = pidBuild.build();
    rightPID = pidBuild.build();
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    romiDrivetrain.resetEncoders();
    leftPID.reset();
    rightPID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    romiDrivetrain.driveUsingSpeeds(leftPID.calculate(romiDrivetrain.getLeftDistanceInch()), rightPID.calculate(romiDrivetrain.getRightDistanceInch()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return leftPID.atSetpoint() && rightPID.atSetpoint();
  }
}
