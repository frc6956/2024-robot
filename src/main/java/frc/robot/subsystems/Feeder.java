package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class Feeder extends SubsystemBase {

  CANSparkFlex topFeeder;
  CANSparkFlex bottomFeeder;

  DigitalInput feedBreak;

  /**
   * The Feeder class represents the subsystem responsible for controlling the feeder mechanism of
   * the robot. It initializes and configures the motors and sensors used by the feeder.
   */
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

  /**
   * Sets the speed of the feeder motors.
   *
   * @param speed the speed at which the feeder motors should run
   */
  public void setSpeed(double speed) {
    topFeeder.set(speed);
    bottomFeeder.set(speed);
  }

  /** Stops the feeder motors. */
  public void stop() {
    topFeeder.set(0);
    bottomFeeder.set(0);
    topFeeder.setInverted(FeederConstants.topInvert);
    bottomFeeder.setInverted(FeederConstants.bottomInvert);
  }

  /**
   * This method is called periodically by the scheduler. It updates the SmartDashboard with the
   * status of the holding note.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Holding Note", holdingNote());
  }

  /**
   * Checks if the feeder is currently holding a note.
   *
   * @return true if the feeder is holding a note, false otherwise
   */
  public boolean holdingNote() {
    return feedBreak.get();
  }
}
