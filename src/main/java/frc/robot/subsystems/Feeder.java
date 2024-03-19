package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;


public class Feeder extends SubsystemBase{

    CANSparkFlex topFeeder;
    CANSparkFlex bottomFeeder;

    DigitalInput feedBreak;

    public Feeder() {
    feedBreak = new DigitalInput(FeederConstants.feedBrakeID);

    topFeeder = new CANSparkFlex(FeederConstants.topFeederID, MotorType.kBrushless);
    bottomFeeder = new CANSparkFlex(FeederConstants.bottomFeederID, MotorType.kBrushless);

    topFeeder.restoreFactoryDefaults();
    bottomFeeder.restoreFactoryDefaults();

    topFeeder.setInverted(FeederConstants.topInvert);
    bottomFeeder.setInverted(FeederConstants.bottomInvert);

    topFeeder.setIdleMode(IdleMode.kCoast);
    bottomFeeder.setIdleMode(IdleMode.kCoast);

    topFeeder.setSmartCurrentLimit(40);
    bottomFeeder.setSmartCurrentLimit(40);

  }

  public void setSpeed(double speed){
    topFeeder.set(speed);
    bottomFeeder.set(speed);
  }

  public void stop(){
    topFeeder.set(0);
    bottomFeeder.set(0);
    topFeeder.setInverted(FeederConstants.topInvert);
    bottomFeeder.setInverted(FeederConstants.bottomInvert);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Holding Note", holdingNote());
  }

  public boolean holdingNote(){
    return feedBreak.get();
  }

    
}
