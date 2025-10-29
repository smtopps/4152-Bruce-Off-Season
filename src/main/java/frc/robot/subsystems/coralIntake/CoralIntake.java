// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coralIntake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants.CoralIntakeAction;
import frc.robot.subsystems.elevator.Elevator;
import org.littletonrobotics.junction.Logger;

public class CoralIntake extends SubsystemBase {

    private final CoralIntakeIO io;
    private CoralIntakeInputsAutoLogged inputs;
    private final Elevator elevator;
    private final Arm arm;

    private Current currentLimit = Amps.of(0);
    /** Creates a new Intake. */
    public CoralIntake(CoralIntakeIO io, Elevator elevator, Arm arm) {
        this.io = io;
        this.elevator = elevator;
        this.arm = arm;
        inputs = new CoralIntakeInputsAutoLogged();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        this.io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public void addSimulatedCoral() {
        this.io.addSimulatedCoral();
    }

    public Command run(CoralIntakeAction intakeAction) {
        return Commands.run(() -> {}, this).beforeStarting(() -> {
            io.setCurrentLimit(intakeAction.currentLimit);
            io.setSpeed(intakeAction.speed);
        });
    }

    public void runIntake(CoralIntakeAction intakeAction) {
        if (intakeAction.currentLimit != currentLimit) {
            io.setCurrentLimit(intakeAction.currentLimit);
            currentLimit = intakeAction.currentLimit;
        }
        io.setSpeed(intakeAction.speed);
    }

    public Command runWithSensor(CoralIntakeAction intakeAction) {
        return run(intakeAction).until(() -> {
            return intakeAction.speed.in(RevolutionsPerSecond) >= 0 ? this.hasCoral() : !this.hasCoral();
        });
    }

    public void stopIntake() {
        this.io.stop();
    }

    public boolean hasCoral() {
        return inputs.hasCoral;
    }

    public void visualizeCoralInIntake(Pose2d robotPose) {
        Pose3d coralPose = new Pose3d();
        coralPose = coralPose.transformBy(new Transform3d(
                Inches.of(11.625),
                Inches.of(0),
                Inches.of(7.8),
                new Rotation3d(Degrees.of(0), Degrees.of(90), Degrees.of(0))));
        coralPose = coralPose.rotateBy(new Rotation3d(
                Degrees.of(0),
                arm.getArmPosition().times(-1).plus(Degrees.of(13)),
                robotPose.getRotation().getMeasure()));
        coralPose = new Pose3d(
                coralPose.getMeasureX(),
                coralPose.getMeasureY(),
                coralPose.getMeasureZ().plus(Inches.of(21.875).plus(elevator.getPosition())),
                coralPose.getRotation());
        coralPose = new Pose3d(
                coralPose.getMeasureX().plus(robotPose.getMeasureX()),
                coralPose.getMeasureY().plus(robotPose.getMeasureY()),
                coralPose.getMeasureZ(),
                coralPose.getRotation());

        Logger.recordOutput("Intake/CoralInIntake", inputs.hasCoral ? new Pose3d[] {coralPose} : new Pose3d[0]);
    }
}
