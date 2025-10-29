// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import java.util.Collections;
import java.util.List;

/** Add your docs here. */
public class MechanismConstants {
    public static final Distance intakeSlopeRate =
            Inches.of(1.5); // distance to go down per 4.5in of coral infront of loading station

    public enum AlgaeLevel {
        ALGAE_L2,
        ALGAE_L3
    }
    // add deadline for intaking so it automatically gpoes to hold for austin

    // fix elevaot zeroing, move arm
    public enum SuperStructurePose {
        // constants call the constructor

        // Elevator position, arm angle
        TROUGH(Inches.of(7.5), Degrees.of(-60)),
        POST_TROUGH(Inches.of(12), Degrees.of(-60)),
        POST_POST_TROUGH(Inches.of(16.75), Degrees.of(-60)),
        STAGED_ALGAE(Inches.of(0.0), Degrees.of(2)),
        // TROUGH(Inches.of(0), Degrees.of(135)),
        // POST_TROUGH(Inches.of(0), Degrees.of(110)),
        // TROUGH(Inches.of(0), Degrees.of(120)),
        LOADING(Inches.of(16.75), Degrees.of(-35)), // 18
        MAX_LOADING(Inches.of(17.85), Degrees.of(-35)),
        MIN_LOADING(Inches.of(0.0), Degrees.of(-20)),
        LOADING_CORAL_BETW(Inches.of(17), Degrees.of(-35)),
        PROCESSOR(Inches.of(0.0), Degrees.of(0)),
        L2(Inches.of(11.5), Degrees.of(135.0)), // 13.0
        L3(Inches.of(27.25), Degrees.of(135.0)), // 15.75 more than L2
        L4(Inches.of(52.7), Degrees.of(146.0)), // 143

        L2_ALGAE(Inches.of(4.0), Degrees.of(160)),
        L2_ALGAE_GRAB(Inches.of(7.25), Degrees.of(160)),
        L2_ALGAE_REMOVE(Inches.of(0.0), Degrees.of(103)),
        L3_ALGAE(Inches.of(19.75), Degrees.of(160)), // 15.75 more than L2 20.75
        L3_ALGAE_GRAB(Inches.of(23.0), Degrees.of(160)), // 15.75 more than L2 25.75
        L3_ALGAE_REMOVE(Inches.of(9.25), Degrees.of(103)), // 15.75 more than L2 10.75

        // ALGAE_PRE_NET(Inches.of(52.7), Degrees.of(103)),
        ALGAE_NET(Inches.of(52.7), Degrees.of(83)),
        ZERO(Inches.of(0.0), Degrees.of(103)),
        HOLD(Inches.of(3.0), Degrees.of(103)),
        CLIMB_STOW(Inches.of(0.0), Degrees.of(-15)),
        BASE(Inches.of(16.75), Degrees.of(-35)); // 3.0, -25

        public final Distance elevatorPosition;
        public final Angle armAngle;

        SuperStructurePose(Distance elevatorPosition, Angle armAngle) {
            this.elevatorPosition = elevatorPosition;
            this.armAngle = armAngle;
        }
    }

    public static List<SuperStructurePose> getAlgaePoses(AlgaeLevel algaeLevel) {
        switch (algaeLevel) {
            case ALGAE_L2:
                return List.of(
                        SuperStructurePose.L2_ALGAE,
                        SuperStructurePose.L2_ALGAE_GRAB,
                        SuperStructurePose.L2_ALGAE_REMOVE);
            case ALGAE_L3:
                return List.of(
                        SuperStructurePose.L3_ALGAE,
                        SuperStructurePose.L3_ALGAE_GRAB,
                        SuperStructurePose.L3_ALGAE_REMOVE);
            default:
                return Collections.emptyList();
        }
    }
}
