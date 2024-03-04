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

public class AutoShoot extends Command {
  /** Creates a new AutoIntake. */
  Intake intake;
  Feeder feeder;
  Wrist wrist;

  int count = 0;
  boolean shotNote = false;

  public AutoShoot(Intake intake, Feeder feeder, Wrist wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.feeder = feeder;
    this.wrist = wrist;
    addRequirements(intake, wrist, feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shotNote = false;
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(wrist.onTarget()){
      if (intake.getRPM() >= IntakeConstants.shootRPM){
          feeder.setSpeed(FeederConstants.feedSpeed);
          count++;
        }
        
    } else {
      wrist.holdWrist(WristConstants.SUBWOOFER);
      feeder.setSpeed(0);
    }
    
    intake.setSpeed(IntakeConstants.shootSpeed);

    if (count > 10){
      shotNote = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shotNote;
  }
}
