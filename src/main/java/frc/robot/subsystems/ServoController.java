// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoController extends SubsystemBase {
  private final Servo servo;

  /** Creates a new ServoController. */
  public ServoController() {
    servo = new Servo(2);
  }

  public void set(int position) {
    servo.set(position);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
