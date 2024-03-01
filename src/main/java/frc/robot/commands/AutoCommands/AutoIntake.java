// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class AutoIntake extends Command {
  /** Creates a new AutoIntake. */
  Intake intake;
  Feeder feeder;
  Wrist wrist;

  boolean noteSecured = false;
  boolean gotNote = false;

  public AutoIntake(Intake intake, Feeder feeder, Wrist wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.feeder = feeder;
    this.wrist = wrist;
    addRequirements(intake, wrist, feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    noteSecured = false;
    gotNote = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(wrist.getDegrees() > 180 && intake.hasNote()){ 
      intake.setSpeed(IntakeConstants.intakeSpeed);
      feeder.setSpeed(IntakeConstants.intakeSpeed);
      wrist.setOutput(0);
    } else if (wrist.getDegrees() < 180 && intake.hasNote()){
      wrist.holdWrist(WristConstants.STOW);
      intake.setSpeed(0);
      feeder.setSpeed(0);
      gotNote = true;
    } else if (wrist.getDegrees() < 180 && !intake.hasNote() && !gotNote){ 
      wrist.holdWrist(WristConstants.PICKUP);
      intake.setSpeed(IntakeConstants.intakeSpeed);
      feeder.setSpeed(0);
    } else {
      intake.setSpeed(0);
      wrist.setSpeed(0);
      feeder.setSpeed(0);
      noteSecured = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return noteSecured;
  }
}
