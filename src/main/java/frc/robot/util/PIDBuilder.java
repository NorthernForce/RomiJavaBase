// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;

/** Add your docs here. */
public class PIDBuilder {
    private final double MkP, MkI, MkD, Mtolerance, MsetPoint;

    public PIDBuilder(double kP, double kI, double kD, double tolerance, double setPoint) {
        this.MkP=kP;
        this.MkI=kI;
        this.MkD=kD;
        this.Mtolerance=tolerance;
        this.MsetPoint=setPoint;
    }

    public PIDController build() {
        PIDController pid = new PIDController(MkP, MkI, MkD);
        pid.setTolerance(Mtolerance);
        pid.setTolerance(MsetPoint);
        return pid;
    }
}
