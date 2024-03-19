// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

public class HoldWrist extends Command {
  /** Creates a new HoldIntake. */
  
  double target;
  Wrist wrist;
  public HoldWrist(Wrist wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wrist = wrist;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    target = wrist.getDegrees();
    wrist.setTarget(target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    target = wrist.getTarget();
    wrist.holdWrist(target);
  }
}
