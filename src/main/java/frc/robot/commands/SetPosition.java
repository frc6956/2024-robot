// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Wrist;

public class SetPosition extends InstantCommand {
  /** Creates a new SetPosition. */
  Wrist wrist;
  double rotation;
  boolean done;
  public SetPosition(Wrist wrist, double rotation) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.wrist = wrist;
    this.rotation = rotation;
    addRequirements(wrist);
    done = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentValue = wrist.getDegrees();

    if (rotation < currentValue && currentValue < WristConstants.PICKUP) {
            rotation = WristConstants.PICKUP;
    } else if (rotation > currentValue && currentValue > WristConstants.STOW) {
            rotation = WristConstants.STOW;
    }

    wrist.holdWrist(rotation);
    if (Math.abs(rotation - currentValue) < 0.1){
      done = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
