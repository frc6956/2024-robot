// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class TeleopIntakeFeed extends Command {
  /** Creates a new TeleopIntake. */
  String status;
  Intake intake;
  Feeder feeder;
  boolean hasNote;
  Wrist wrist;


  public TeleopIntakeFeed(Intake intake, Feeder feeder,String status, Wrist wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake=intake;
    this.feeder=feeder;
    this.status=status;
    this.wrist=wrist;
    addRequirements(intake, feeder);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (status) {
      case "INTAKE":
        if(wrist.getDegrees() > 150){
          intake.setSpeed(IntakeConstants.intakeSpeed);
        } else if (!intake.hasNote()){
          intake.setSpeed(IntakeConstants.intakeSpeed);
        } else intake.stop();
        feeder.setSpeed(IntakeConstants.intakeSpeed);
        break;
      case "EXTAKE":
        intake.setSpeed(IntakeConstants.extakeSpeed);
        feeder.setSpeed(IntakeConstants.extakeSpeed);
        break;
      case "SHOOT":
        if (intake.getRPM() >= IntakeConstants.shootRPM){
          feeder.setSpeed(FeederConstants.feedSpeed);
        }
        intake.setSpeed(IntakeConstants.shootSpeed);
        break;
      case "STOP":
        intake.stop();
        feeder.stop();
        break;
      default:
        intake.stop();
        feeder.stop();
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    feeder.stop();
  }
}
