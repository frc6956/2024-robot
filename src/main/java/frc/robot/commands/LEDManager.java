// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Driver;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs.LEDConstants;
import frc.robot.subsystems.LEDs.LEDs;

public class LEDManager extends Command {
  /** Creates a new LEDManager. */
  int count = 0;
  private LEDs leds;
  public LEDManager(LEDs leds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.leds = leds;
    addRequirements(leds);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leds.setRainbow(leds.getAllLEDs());
  }

  public void controlLEDs(){
    switch (getState()) {
      case "DISABLED":
        leds.setRainbow(leds.getPanelsRange());
        break;
      case "AUTONRED":
        leds.setAllPanelColor(LEDConstants.red);
        break;
      case "AUTONBLUE":
        leds.setAllPanelColor(LEDConstants.blue);
        break;
      case "TELEOP":
        leds.togglePanel(leds.getRightPanel(), leds.getLeftPanel(), LEDConstants.shamrock);
        break;
      case "SETGREEN":
        leds.setAllColor(LEDConstants.green);
        break;
      case "OFF":
        leds.setAllOff();
        break;
      default:
        leds.setAllOff();
        break;
    }
  }

  private String getState(){
    if (DriverStation.isDSAttached()){
      if (DriverStation.isDisabled()){
        return "DISABLED";
      } else { 
        /* 
        if (DriverStation.isAutonomous()){
          if (DriverStation.getAlliance().get() == Alliance.Blue){
            return "AUTONBLUE";
          } else if (DriverStation.getAlliance().get() == Alliance.Red){
            return "AUTONRED";
          } else {
            return "SETGREEN";
          }
        }
        */
        return "TELEOP";
      }
    } else return "OFF";
  }

  @Override 
  public boolean runsWhenDisabled(){
    return true;
  }
}
