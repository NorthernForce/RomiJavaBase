// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeamBrake extends SubsystemBase {
  private final DigitalInput transmitter;
  private final DigitalInput receiver;

  /** Creates a new BeamBrake. */
  public BeamBrake() {
    transmitter = new DigitalInput(8);
    receiver = new DigitalInput(9);
  }

  public boolean get() {
    return receiver.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
