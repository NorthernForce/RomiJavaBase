// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RomiGyro extends SubsystemBase {
  private SimDouble rateX, rateY, rateZ, angleX, angleY, angleZ;
  private double offset = (90.0 / 175 + 2.48 / 90) * 0.01;
  private double xOffset, yOffset, zOffset;
  private SimDevice gyro;
  /** Creates a new RomiGyro. */
  public RomiGyro() {
    gyro = SimDevice.create("Gyro:RomiGyro");
    assert gyro != null;
    gyro.createBoolean("init", Direction.kOutput, true);
    rateX = gyro.createDouble("rate_x", Direction.kInput, 0.0);
    rateY = gyro.createDouble("rate_y", Direction.kInput, 0.0);
    rateZ = gyro.createDouble("rate_z", Direction.kInput, 0.0);
    angleX = gyro.createDouble("angle_x", Direction.kInput, 0.0);
    angleY = gyro.createDouble("angle_y", Direction.kInput, 0.0);
    angleZ = gyro.createDouble("angle_z", Direction.kInput, 0.0);
    xOffset = 0.0;
    yOffset = 0.0;
    zOffset = 0.0;
  }
  public double getRateX()
  {
    return rateX.get();
  }
  public double getRateY()
  {
    return rateY.get();
  }
  public double getRateZ()
  {
    return rateZ.get();
  }
  public double getAngleX()
  {
    return angleX.get() - xOffset;
  }
  public double getAngleY()
  {
    return angleY.get() - yOffset;
  }
  public double getAngleZ()
  {
    return angleZ.get() - zOffset;
  }
  @Override
  public void periodic()
  {
    zOffset += offset;
  }
  public void reset()
  {
    xOffset = angleX.get();
    yOffset = angleY.get();
    zOffset = angleZ.get();
  }
  public void calibrate()
  {
    offset = 0;
    System.out.println("Calibrating. Do Not Move.");
    double startAngle = getAngleZ();
    Timer.delay(10);
    double endAngle = getAngleZ();
    offset = (endAngle - startAngle) / (10.0 * 50);
    reset();
  }
}
