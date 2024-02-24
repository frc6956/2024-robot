// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class TeleopIntake extends Command {
  /** Creates a new TeleopIntake. */
  String status;
  Intake intake;
  boolean hasNote;

  public TeleopIntake(Intake intake, String status) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake=intake;
    this.status=status;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hasNote = intake.hasNote();
    switch (status) {
      case "INTAKE":
        intake.setSpeed(IntakeConstants.intakeSpeed);
        break;
    
      default:
        intake.stop();
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
