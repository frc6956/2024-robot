// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConstants;
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
    if(intake.hasNote()) {
      gotNote = true;
    }

    if(wrist.getDegrees() >= WristConstants.INTAKEGOOD && intake.hasNote()){ 
      intake.setSpeed(IntakeConstants.intakeSpeed);
      feeder.setSpeed(FeederConstants.intakeSpeed);
      wrist.holdWrist(WristConstants.STOW);
    } else if (wrist.getDegrees() < WristConstants.INTAKEGOOD && intake.hasNote()){
      wrist.holdWrist(WristConstants.STOW);
      intake.stop();
      feeder.stop();
      gotNote = true;
    } else if (wrist.getDegrees() < WristConstants.INTAKEGOOD && !intake.hasNote() && !gotNote){ 
      wrist.holdWrist(WristConstants.PICKUP);
      intake.setSpeed(IntakeConstants.intakeSpeed);
      feeder.stop();
    } else if (wrist.getDegrees() >= WristConstants.INTAKEGOOD){
      if (feeder.holdingNote()){
        noteSecured = true;
      } else {
        intake.setSpeed(IntakeConstants.intakeSpeed);
        feeder.setSpeed(FeederConstants.intakeSpeed);
        if(gotNote) {
          wrist.holdWrist(WristConstants.STOW);
        } else {
          wrist.holdWrist(WristConstants.PICKUP);
        }
      }
    }
    else {
      intake.stop();
      wrist.holdWrist(WristConstants.STOW);
      feeder.stop();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.holdWrist(WristConstants.STOW);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return noteSecured;
  }
}
