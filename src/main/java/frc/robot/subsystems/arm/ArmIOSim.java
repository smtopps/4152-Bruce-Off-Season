// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class ArmIOSim implements ArmIO {
    final TalonFX armMotor = new TalonFX(32, "rio");
    final CANcoder armEncoder = new CANcoder(33, "rio");

    final TalonFXSimState armMotorSim = armMotor.getSimState();
    final CANcoderSimState armEncoderSim = armEncoder.getSimState();

    double armRatio = 119.5887; // 185.7143;
    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            armRatio,
            0.16349933,
            // SingleJointedArmSim.estimateMOI(0.2897255217, 0.2897255217),
            0.2897255217,
            ArmConstants.baseAngle.in(Radians),
            Units.degreesToRadians(180.0),
            true,
            ArmConstants.startingAngle.in(Radians));

    private final MotionMagicVoltage magicRequest = new MotionMagicVoltage(0.0);

    public ArmIOSim() {
        configureArmMotor();
    }

    @Override
    public void updateInputs(ArmInputs inputs) {
        armMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        armEncoderSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        armSim.setInputVoltage(armMotorSim.getMotorVoltage());
        armSim.update(0.02);

        final var armAngle = Angle.ofBaseUnits(armSim.getAngleRads(), Radians);
        final var armVelocity = AngularVelocity.ofBaseUnits(armSim.getVelocityRadPerSec(), RadiansPerSecond);

        armEncoderSim.setRawPosition(armAngle);
        armEncoderSim.setVelocity(armVelocity);
        armMotorSim.setRawRotorPosition(armAngle.times(armRatio));
        armMotorSim.setRotorVelocity(armVelocity.times(armRatio));

        inputs.armAngle = armMotor.getPosition(true).getValue();
    }

    @Override
    public void setPosition(Angle targetAngle) {
        armMotor.setControl(magicRequest.withPosition(targetAngle).withSlot(0));
    }

    @Override
    public void stop() {
        armMotor.stopMotor();
    }

    private void configureArmMotor() {
        CANcoderConfiguration armEncoderConfig = new CANcoderConfiguration();
        armEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.8;
        // armEncoderConfig.MagnetSensor.MagnetOffset = 0.117;
        armEncoderConfig.MagnetSensor.MagnetOffset = 0.075439;
        armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        armEncoder.getConfigurator().apply(armEncoderConfig);
        TalonFXConfiguration armConfig = new TalonFXConfiguration();
        armConfig.CurrentLimits.StatorCurrentLimit = 30; // 60
        armConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        armConfig.Feedback.FeedbackRemoteSensorID = 33;
        armConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        armConfig.Feedback.RotorToSensorRatio = armRatio;
        armConfig.MotionMagic.MotionMagicAcceleration = 4.0; // 6.0
        armConfig.MotionMagic.MotionMagicCruiseVelocity = 1.0; // 2.0
        armConfig.MotionMagic.MotionMagicJerk = 0.0;
        armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        armConfig.Slot0.kG = 0.15; // 0.0
        armConfig.Slot0.kS = 0.1; // 0.15;
        armConfig.Slot0.kD = 2.0; // 0.08;
        armConfig.Slot0.kV = 14.2222; // 23.0; // 21.04111;
        armConfig.Slot0.kA = 0.0;
        armConfig.Slot0.kP = 80.0; // 80; // 45
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(200.0);
        armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        // armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.12;
        armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(-65.0);
        armConfig.Voltage.PeakForwardVoltage = 10.0;
        armConfig.Voltage.PeakReverseVoltage = 10.0;

        armMotor.getConfigurator().apply(armConfig);

        armMotorSim.Orientation = ChassisReference.Clockwise_Positive;
        armEncoderSim.Orientation = ChassisReference.Clockwise_Positive;
    }
}
