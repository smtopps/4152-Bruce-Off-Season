// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coralIntake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

/** Add your docs here. */
public class CoralIntakeConstants {
    public enum CoralIntakeAction {
        HOLDING(Amps.of(15), RevolutionsPerSecond.of(4.0)), // 15
        INTAKING(Amps.of(20), RevolutionsPerSecond.of(15.0)),
        TROUGH(Amps.of(30), RevolutionsPerSecond.of(-8.0)),
        PLACING(Amps.of(40), RevolutionsPerSecond.of(-8.0)),
        PLACING_L4(Amps.of(40), RevolutionsPerSecond.of(-25.0)),
        EMPTY(Amps.of(10), RevolutionsPerSecond.of(0.0));

        public final Current currentLimit;
        public final AngularVelocity speed;

        CoralIntakeAction(Current currentLimit, AngularVelocity speed) {
            this.currentLimit = currentLimit;
            this.speed = speed;
        }
    }

    public static final double PlacingTimeout = 0.25; // 0.35
    public static final double PostSensingTimeout = 0.12; // 0.08
    public static final double PostPlacingTimeout = 0.35;
}
