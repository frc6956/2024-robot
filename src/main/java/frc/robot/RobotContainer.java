// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PhotonVision;

import frc.robot.commands.drivetrain.*;
import frc.robot.auton.common.*;
import frc.utils.OnTheFlyPathing;



/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

	public static final double GAMEPAD_AXIS_THRESHOLD = 0.15;
	public static final double JOYSTICK_AXIS_THRESHOLD = 0.15;

	public static final int LX = 0;
	public static final int LY = 1;
	public static final int LT = 2;
	public static final int RT = 3;
	public static final int RX = 4;
	public static final int RY = 5;

	// choosers (for auton)
	
	public static final String AUTON_DO_NOTHING = "Do Nothing";
	public static final String AUTON_CUSTOM = "My Auto";
	public static final String AUTON_SAMPLE_SWERVE = "Sample Swerve";
	public static final String AUTON_SAMPLE_MOVE_FORWARD = "Sample Move Forward";
	public static final String AUTON_SAMPLE_MOVE_IN_REVERSE = "Sample Move In Reverse";
	public static final String AUTON_SAMPLE_MOVE_IN_GAMMA_SHAPE = "Sample Move In Gamma Shape";
	public static final String AUTON_SAMPLE_MOVE_IN_L_SHAPE_IN_REVERSE = "Sample Move In L Shape In Reverse";
	//private String autonSelected;
	private SendableChooser<Command> autonChooser = new SendableChooser<>();
	


	// motorized devices
	private final Pigeon2 gyro = new Pigeon2(0);
	private final SwerveDrivetrain drivetrain = new SwerveDrivetrain(gyro);
	protected Limelight m_limelight = null;
	protected PhotonVision m_photonVision = null;
	
	// pneumatic devices

	// misc

	private final Field2d field = new Field2d(); //  a representation of the field


	// The driver's and copilot's joystick(s) and controller(s)

	/*CommandJoystick joyLeft = new CommandJoystick(Ports.USB.LEFT_JOYSTICK);
	CommandJoystick joyRight = new CommandJoystick(Ports.USB.RIGHT_JOYSTICK);*/
	//CommandJoystick joyMain = new CommandJoystick(Ports.USB.MAIN_JOYSTICK);
	CommandXboxController driveController = new CommandXboxController(Ports.USB.DRIVER_GAMEPAD);
	CommandXboxController copilotGamepad = new CommandXboxController(Ports.USB.COPILOT_GAMEPAD);
	

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		// choosers (for auton)
		
		/*autonChooser.setDefaultOption("Do Nothing", AUTON_DO_NOTHING);
		autonChooser.addOption("My Auto", AUTON_CUSTOM);
		autonChooser.addOption("Sample Swerve", AUTON_SAMPLE_SWERVE);
		autonChooser.addOption("Sample Move Forward", AUTON_SAMPLE_MOVE_FORWARD);
		autonChooser.addOption("Sample Move In Reverse", AUTON_SAMPLE_MOVE_IN_REVERSE);
		autonChooser.addOption("Sample Move In Gamma Shape", AUTON_SAMPLE_MOVE_IN_GAMMA_SHAPE);
		autonChooser.addOption("Sample Move In L Shape In Reverse", AUTON_SAMPLE_MOVE_IN_L_SHAPE_IN_REVERSE);
		SmartDashboard.putData("Auto choices", autonChooser);*/
		

		// Configure the button bindings

		configureButtonBindings();

		autonChooser = AutoBuilder.buildAutoChooser("Default");
        SmartDashboard.putData("Auto Choices", autonChooser);


		// Configure default commands

		drivetrain.setDefaultCommand(
			// The left stick controls translation of the robot.
			// Turning is controlled by the X axis of the right stick.
			// We are inverting LeftY because Xbox controllers return negative values when we push forward.
			// We are inverting LeftX because we want a positive value when we pull to the left. Xbox controllers return positive values when you pull to the right by default.
			// We are also inverting RightX because we want a positive value when we pull to the left (CCW is positive in mathematics).
			new RunCommand(
				() -> drivetrain.drive(
					-MathUtil.applyDeadband(driveController.getLeftY(), JOYSTICK_AXIS_THRESHOLD),
					-MathUtil.applyDeadband(driveController.getLeftX(), JOYSTICK_AXIS_THRESHOLD),
					-MathUtil.applyDeadband(driveController.getRightX(), JOYSTICK_AXIS_THRESHOLD),
					true, true),
				drivetrain));

	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
	 * subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
	 * passing it to a
	 * {@link JoystickButton}.
	 */
	private void configureButtonBindings() {

		// driver (joystick)

		driveController.povUp()
			.onTrue(new DrivetrainZeroHeading(drivetrain));	

			//driveController.povDown()
			//.onTrue(new DrivetrainOppositeHeading(drivetrain));	

			driveController.povDown()
			.whileTrue(new DrivetrainSetXFormation(drivetrain));	

			driveController.button(1);

			driveController.y()
			.whileTrue(
				new OnTheFlyPathing().getOnTheFlyPath(0, 0)
			  );
		
			
			
			//driveController.button(3)
			//.onTrue(new MoveInLShapeInReverse(drivetrain, this, 3));
			
			//driveController.button(4)
			//.onTrue(new MoveInGammaShape(drivetrain, this, 3));

			//driveController.button(5)
			//.onTrue(new MoveForward(drivetrain, this, 3));
			//.onTrue(new DrivetrainTurnAngleUsingPidController(drivetrain, -90));
			//.onTrue(new MoveInUShapeInReverse(drivetrain, this, 1));

			driveController.button(2);
			//.onTrue(new MoveInReverse(drivetrain, this, 3));
			//.onTrue(new DrivetrainTurnAngleUsingPidController(drivetrain, 90));


/* 
		// copilot (gamepad)
		
		copilotGamepad.a()
			.whileTrue(new RollerRelease(roller));
		
		copilotGamepad.b()
			.whileTrue(new RollerRoll(roller));

		copilotGamepad.x()
			.onTrue(new MouthSafeClose(mouth, neck, getCopilotGamepad()));

		copilotGamepad.y()
			.onTrue(new MouthOpen(mouth));

		copilotGamepad.back()
			.onTrue(new DrivetrainAndGyroReset(drivetrain));

		copilotGamepad.start()
			.onTrue(new AlmostEverythingStop(elevator, drawer, neck, roller));


		copilotGamepad.leftTrigger()
			.onTrue(new DrawerRetractWithStallDetection(drawer));

		copilotGamepad.rightTrigger()
			.onTrue(new DrawerExtendWithStallDetection(drawer));


		copilotGamepad.povDown()
			.onTrue(new ElevatorMoveDownWithStallDetection(elevator));

		copilotGamepad.povLeft()
			.onTrue(new ElevatorMoveMidwayWithStallDetection(elevator));

		copilotGamepad.povRight()
			.onTrue(new ElevatorMoveMidwayWithStallDetection(elevator));

		copilotGamepad.povUp()
			.onTrue(new ElevatorMoveUpWithStallDetection(elevator));


		copilotGamepad.leftBumper()
			.onTrue(new NeckSafeMoveUpWithStallDetection(neck, mouth, getCopilotGamepad()));

		copilotGamepad.rightBumper()
			.onTrue(new NeckMoveDownWithStallDetection(neck));


		copilotGamepad.leftStick()
			.onTrue(new RollerTimedRoll(roller, 3));
			//.onTrue(new GamepadRumble(getCopilotGamepad(),false));			

		copilotGamepad.rightStick()
			.onTrue(new RollerTimedRelease(roller, 3));
			//.onTrue(new GamepadRumble(getCopilotGamepad(),false));


		copilotGamepad.axisGreaterThan(LY,GAMEPAD_AXIS_THRESHOLD)
			.whileTrue(new ElevatorGamepadControl(elevator, getCopilotGamepad()));

		copilotGamepad.axisLessThan(LY,-GAMEPAD_AXIS_THRESHOLD)
			.whileTrue(new ElevatorGamepadControl(elevator, getCopilotGamepad()));
*/
		/*copilotGamepad.axisGreaterThan(LX,GAMEPAD_AXIS_THRESHOLD)
			.whileTrue();

		copilotGamepad.axisLessThan(LX,-GAMEPAD_AXIS_THRESHOLD)
			.whileTrue();*/
/*
		copilotGamepad.axisGreaterThan(RY,GAMEPAD_AXIS_THRESHOLD)
			.whileTrue(new NeckGamepadControl(neck, getCopilotGamepad()));

		copilotGamepad.axisLessThan(RY,-GAMEPAD_AXIS_THRESHOLD)
			.whileTrue(new NeckGamepadControl(neck, getCopilotGamepad()));

		copilotGamepad.axisGreaterThan(RX,GAMEPAD_AXIS_THRESHOLD)
			.whileTrue(new DrawerGamepadControl(drawer, getCopilotGamepad()));

		copilotGamepad.axisLessThan(RX,-GAMEPAD_AXIS_THRESHOLD)
			.whileTrue(new DrawerGamepadControl(drawer, getCopilotGamepad()));	
*/			
	}


	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		/*autonSelected = autonChooser.getSelected();
		System.out.println("Auton selected: " + autonSelected);
		
		switch (autonSelected) {
			case AUTON_SAMPLE_SWERVE:
				//return createSwerveControllerCommand(createExampleTrajectory());
				//return new DrivetrainSwerveRelative(drivetrain, this, createExampleTrajectory());
				return new MoveInSShape(drivetrain, this, 3);
				//break;

			case AUTON_SAMPLE_MOVE_FORWARD:
				return new MoveForward(drivetrain, this, 3);
				//break;

			case AUTON_SAMPLE_MOVE_IN_REVERSE:
				return new MoveInReverse(drivetrain, this, 3);
				//break;

			case AUTON_SAMPLE_MOVE_IN_GAMMA_SHAPE:
				return new MoveInGammaShape(drivetrain, this, 3);
				//break;

			case AUTON_SAMPLE_MOVE_IN_L_SHAPE_IN_REVERSE:
				return new MoveInLShapeInReverse(drivetrain, this, 3);
				//break;
			case AUTON_DO_NOTHING:
				return null;
				//break;
				
			default:
				// nothing
				return null;
				//break;
		} // end switch*/
		return autonChooser.getSelected();
	}

	public TrajectoryConfig createTrajectoryConfig() {
		// Create config for trajectory
		TrajectoryConfig config = new TrajectoryConfig(
			AutoConstants.MAX_SPEED_METERS_PER_SECOND,
			AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
			// Add kinematics to ensure max speed is actually obeyed
			.setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);

		return config;
	}

	public TrajectoryConfig createReverseTrajectoryConfig() {

		TrajectoryConfig config = createTrajectoryConfig();

		config.setReversed(true); // in reverse!

		return config;
	}

	public Field2d getField()
	{
		return field;
	}

	public SwerveDrivetrain getDrivetrain()
	{
		return drivetrain;
	}

	public XboxController getCopilotGamepad()
	{
		return driveController.getHID();
	}

	public SendableChooser<Command> getAutonChooser()
	{
		return autonChooser;
	}

	public double getX(){
		return driveController.getLeftX();
	}

	public double getY(){
		return driveController.getLeftY();
	}

	public double getRot(){
		return driveController.getRightX();
	}

	public Limelight getLimelight() {
		return m_limelight;
	  }
	
	  public PhotonVision getPhotonVision() {
		return m_photonVision;
	  }
}