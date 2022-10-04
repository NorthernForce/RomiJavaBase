// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class OI {
    private final static XboxController driverController = new XboxController(0);

    public static DoubleSupplier[] getDriveSuppliers() {
        return new DoubleSupplier[] {() -> -driverController.getLeftY(), () -> driverController.getRightX()};
     }
}
