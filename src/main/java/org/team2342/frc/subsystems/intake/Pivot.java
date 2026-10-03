// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team2342.frc.Constants.IntakeConstants;
import org.team2342.frc.util.ExecutionLogger;
import org.team2342.lib.motors.smart.SmartMotorIO;
import org.team2342.lib.motors.smart.SmartMotorIOInputsAutoLogged;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

public class Pivot extends SubsystemBase {

  private final SmartMotorIO pivotMotor;
  private final SmartMotorIOInputsAutoLogged pivotMotorInputs = new SmartMotorIOInputsAutoLogged();

  @AutoLogOutput(key = "Intake/Pivot/TargetAngle")
  private double goal = IntakeConstants.MAX_ANGLE;

  private final Alert pivotMotorAlert =
      new Alert("intakePivot", "Intake Pivot motor is disconnected!", Level.HIGH);

  public Pivot(SmartMotorIO pivotMotor) {
    this.pivotMotor = pivotMotor;
    setName("Intake/Pivot");
    setDefaultCommand(run(() -> pivotMotor.runPosition(goal)));
  }

  @Override
  public void periodic() {
    pivotMotor.updateInputs(pivotMotorInputs);
    Logger.processInputs("Intake/Pivot", pivotMotorInputs);
    pivotMotorAlert.set(!pivotMotorInputs.motorsConnected[0]);
    ExecutionLogger.log("Intake/Pivot");
  }

  public void runAngle(double angle) {
    goal = Math.clamp(angle, IntakeConstants.MIN_ANGLE, IntakeConstants.MAX_ANGLE);
    pivotMotor.runPosition(goal);
  }

  public Command goToAngle(double angle) {
    return run(() -> runAngle(angle))
        .until(() -> Math.abs(goal - angle) <= 0.01)
        .withName("Pivot Go To Angle");
  }

  public void freeze() {
    goal = pivotMotorInputs.positionRad;
  }

  public Command runVoltage(double voltage) {
    return run(() -> pivotMotor.runVoltage(voltage)).withName("Pivot Run Voltage");
  }

  public Command holdAngle(double angle) {
    return run(() -> runAngle(angle)).withName("Pivot Hold Angle");
  }

  public Command stop() {
    return runOnce(() -> pivotMotor.runVoltage(0.0)).withName("Pivot Stop");
  }

  public Command agitate() {
    return Commands.repeatingSequence(
            holdAngle(1.4).withTimeout(0.5), holdAngle(0.5).withTimeout(0.5))
        .withName("Pivot Agitate");
  }
}
