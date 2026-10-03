// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.indexer;

import org.littletonrobotics.junction.Logger;
import org.team2342.frc.Constants.IndexerConstants;
import org.team2342.frc.util.ExecutionLogger;
import org.team2342.lib.motors.dumb.DumbMotorIO;
import org.team2342.lib.motors.dumb.DumbMotorIOInputsAutoLogged;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

public class Indexer extends SubsystemBase {
  private final DumbMotorIO motor;
  private final DumbMotorIOInputsAutoLogged inputs = new DumbMotorIOInputsAutoLogged();

  private final Alert motorAlert = new Alert("indexer", "Indexer Motor is diconnected", Level.HIGH);

  public Indexer(DumbMotorIO motor) {
    this.motor = motor;
    setName("Indexer");

    setDefaultCommand(run(() -> motor.runVoltage(0.0)));
  }

  @Override
  public void periodic() {
    motor.updateInputs(inputs);

    Logger.processInputs("Indexer/BeltMotor", inputs);

    motorAlert.set(!inputs.connected);

    ExecutionLogger.log("Indexer");
  }

  public Command in() {
    return run(() -> motor.runVoltage(IndexerConstants.RUN_VOLTAGE)).withName("Indexer Feed");
  }

  public Command out() {
    return run(() -> motor.runVoltage(-IndexerConstants.RUN_VOLTAGE)).withName("Indexer Out");
  }

  public Command pulseIn() {
    return Commands.repeatingSequence(
        in().withTimeout(2), stop().andThen(Commands.waitSeconds(0.2)));
  }

  public Command stop() {
    return runOnce(() -> motor.runVoltage(0.0)).withName("Indexer Stop");
  }

  public boolean isJammed() {
    return inputs.currentAmps > IndexerConstants.INDEXER_MOTOR_CONFIG.statorLimit;
  }
}
