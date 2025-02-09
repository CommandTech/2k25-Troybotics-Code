// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Coral extends SubsystemBase {
  private SparkMax coralMotor;
  private SparkMaxConfig config;
  /** Creates a new Coral. */
  public Coral() {
      coralMotor = new SparkMax(Constants.MotorConstants.CORAL_MOTOR_ID, SparkMax.MotorType.kBrushless);
      config = new SparkMaxConfig();
      config.inverted(Constants.MotorConstants.CORAL_MOTOR_INVERTED);
      config.smartCurrentLimit(Constants.MotorConstants.CORAL_MOTOR_AMP_LIMIT);

      coralMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //1 to deliver 0 to stop
  public void setMotors(double value) {
    coralMotor.set(value);
  }

  public Command deliverCoral() {
    // Deliver the coral
    return runOnce(() -> setMotors(1));
  }
}
