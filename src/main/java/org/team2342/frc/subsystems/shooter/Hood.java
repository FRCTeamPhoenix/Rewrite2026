// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.shooter;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.team2342.frc.Constants.ShooterConstants;
import org.team2342.frc.util.ExecutionLogger;
import org.team2342.lib.motors.smart.SmartMotorIO;
import org.team2342.lib.motors.smart.SmartMotorIOInputsAutoLogged;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

public class Hood extends SubsystemBase {
  private final SmartMotorIO motor;
  private final SmartMotorIOInputsAutoLogged motorInputs = new SmartMotorIOInputsAutoLogged();

  private final Alert motorAlert =
      new Alert("shooterHood", "Shooter Hood motor is disconnected!", Level.HIGH);

  public Hood(SmartMotorIO motor) {
    this.motor = motor;

    setName("Shooter/Hood");
    setDefaultCommand(run(() -> motor.runVoltage(0.0)));
  }

  @Override
  public void periodic() {
    motor.updateInputs(motorInputs);
    Logger.processInputs("Shooter/Hood", motorInputs);

    motorAlert.set(!motorInputs.motorsConnected[0]);

    ExecutionLogger.log("Shooter/Hood");
  }

  public double getHoodAngle() {
    return motorInputs.positionRad;
  }

  public void runAngle(double targetAngle) {
    double clampedAngle = Math.clamp(targetAngle, 0.0, ShooterConstants.MAX_ANGLE);
    Logger.recordOutput("Shooter/Hood/Setpoint", clampedAngle);

    motor.runPosition(clampedAngle);
  }

  public void runAngle(DoubleSupplier targetAngle) {
    double clampedAngle = Math.clamp(targetAngle.getAsDouble(), 0.0, ShooterConstants.MAX_ANGLE);
    Logger.recordOutput("Shooter/Hood/Setpoint", clampedAngle);

    motor.runPosition(clampedAngle);
  }

  public Command goToAngle(double targetAngle) {
    return run(() -> runAngle(targetAngle))
        .until(
            () ->
                Math.abs(targetAngle - motorInputs.positionRad)
                    <= ShooterConstants.TARGET_TOLERANCE)
        .withName("Hood GoToAngle");
  }

  public Command goToAngle(DoubleSupplier targetAngle) {
    return run(() -> runAngle(targetAngle.getAsDouble()))
        .until(
            () ->
                Math.abs(targetAngle.getAsDouble() - motorInputs.positionRad)
                    <= ShooterConstants.TARGET_TOLERANCE)
        .withName("Hood GoToAngle");
  }

  public Command holdAngle(double targetAngle) {
    return run(() -> runAngle(targetAngle)).withName("Hood HoldAngle");
  }

  public Command holdAngle(DoubleSupplier targetAngle) {
    return run(() -> runAngle(targetAngle)).withName("Hood HoldAngle");
  }

  public Command stop() {
    return runOnce(() -> motor.runVoltage(0.0)).withName("Hood Stop");
  }
}
