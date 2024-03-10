// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

	Joystick Controller = new Joystick(Constants.Joystick_Port);

	// variable of each motors speed
	double[] Motors = { 0, 0 };

	boolean enableArm = true;

	// creates all motor object
	CANSparkBase leftRear;
	CANSparkBase leftFront;
	CANSparkBase rightRear;
	CANSparkBase rightFront;
	CANSparkBase testmotor;

	

	// initializes encoders
	RelativeEncoder Encoder_LB;
	RelativeEncoder Encoder_LF;
	RelativeEncoder Encoder_RB;
	RelativeEncoder Encoder_RF;

	// Premade class initialization
	DifferentialDrive m_drivetrain;

	// refrence variable to check diffrence between positions
	private double Rotation_Reference = 0;

	// initialize gyroscope
	ADIS16470_IMU imu = new ADIS16470_IMU();

	// Autonomous stuff
	private int Action_Number;
	private String Current_Action = "none";
	private Action Action = new Action();
	private final SendableChooser<String> Chosen_Preset = new SendableChooser<>();
	private Arm Launcher = new Arm();

	// PID init
	PID PIDMotorA, PIDMotorB, PIDMotorC, PIDMotorD;

	// Commented out V
	double ThresholdX; // Should be some 0.00...x + Value to account for someone just statically
						// holding the controller.
	double ThresholdY; // Therefore the drift correction works but it cannot be dynamically updated

	// Initilization
	@Override
	public void robotInit() {

		// Variables for joystick drift:
		// MAke drift threshods dynamic through reading value from smartdashboard?

		// FOR THE DASHBOARD
		Chosen_Preset.setDefaultOption("Test", "Test");
		Chosen_Preset.addOption("Circle", "Circle");
		Chosen_Preset.addOption("Turn", "Turn");
		Chosen_Preset.addOption("Arc", "Arc");
		Chosen_Preset.addOption("Die", "Die");
		SmartDashboard.putData("Auto Options", Chosen_Preset);
		SmartDashboard.putNumber("Speed Limit", Constants.SpeedLimit);
		SmartDashboard.putNumber("Acceleration", Constants.Acceleration);
		SmartDashboard.putNumber("Action Number", Action_Number);
		Launcher.armInit(Controller);

		// initializes motors
		leftRear = new CANSparkMax(Constants.LEFT_BACK_ID, MotorType.kBrushless);
		leftFront = new CANSparkMax(Constants.LEFT_FRONT_ID, MotorType.kBrushless);
		rightRear = new CANSparkMax(Constants.RIGHT_BACK_ID, MotorType.kBrushless);
		rightFront = new CANSparkMax(Constants.RIGHT_FRONT_ID, MotorType.kBrushless);

		// initializes encoder
		Encoder_LB = leftRear.getEncoder();
		Encoder_LF = leftFront.getEncoder();
		Encoder_RB = rightRear.getEncoder();
		Encoder_RF = rightFront.getEncoder();
		

		// limits amps of drive motors
		leftRear.setSmartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT_A);
		leftFront.setSmartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT_A);
		rightRear.setSmartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT_A);
		rightFront.setSmartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT_A);

		// We need to invert one side of the drivetrain because motor flipped
		leftFront.setInverted(false);
		rightFront.setInverted(true);

		// Tells the rear wheels to follow the same commands as the front wheels
		leftRear.follow(leftFront);
		rightRear.follow(rightFront);

		// create drivetrain
		m_drivetrain = new DifferentialDrive(leftFront, rightFront);
		

		// calibrate gyroscope
		imu.calibrate();
	}

	@Override
	public void robotPeriodic() {
		// dashboard dsiplay data
		SmartDashboard.putString("action", Current_Action);
		SmartDashboard.putData("Differential Drivebase", m_drivetrain);
		SmartDashboard.putNumber("Time Elapsed", Timer.getFPGATimestamp());
		SmartDashboard.putNumber("Gyro", (360 - imu.getAngle(imu.getYawAxis())));
		SmartDashboard.putData("Auto Options", Chosen_Preset);
		SmartDashboard.putNumber("ActionNumb", Action_Number);

		if (Action.goal.length != 0) {
			switch (Action.action[Action_Number]) {
				case "drive":
					SmartDashboard.putString("Distance to Goal",
							"" + (Action.goal[Action_Number][0] - calculateMovement()));
					break;
				case "turn":
					SmartDashboard.putString("Distance to Goal", "Degrees: "
							+ (Action.goal[Action_Number][0] - imu.getAngle(imu.getYawAxis()) + Rotation_Reference));
					break;
				case "arc":
					SmartDashboard.putString("Distance to Goal", "Degrees: "
							+ (Action.goal[Action_Number][0] - imu.getAngle(imu.getYawAxis()) + Rotation_Reference));
					break;
			}
		} else {
			SmartDashboard.putString("Distance to Goal", "N/A");
		}

		// dashboard retrieve data
		Constants.SpeedLimit = SmartDashboard.getNumber("Speed Limit", 0.5);
		Constants.Acceleration = SmartDashboard.getNumber("Acceleration", 0.01);
		ThresholdX = SmartDashboard.getNumber("ThresholdX", 0.01); // Consider default values for drift correction.
		ThresholdY = SmartDashboard.getNumber("ThresholdY", 0.01); // Default be some 0.00...x + Value to account for
																	// someone just statically holding the controller.
																	// Then, update dynmaically

	}

	/** This function is run once each time the robot enters autonomous mode. */
	@Override
	public void autonomousInit() {
		// sets motors to brake
		leftRear.setIdleMode(IdleMode.kBrake);
		leftFront.setIdleMode(IdleMode.kBrake);
		rightRear.setIdleMode(IdleMode.kBrake);
		rightFront.setIdleMode(IdleMode.kBrake);
		// resets auto stuff
		imu.reset();
		Rotation_Reference = 0;
		setBaseLocation();
		Action_Number = 0;

		// add the actions taken during autonomous
		Action.LoadPreset(Chosen_Preset);
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
		// selects current action
		Current_Action = Action.action[Action_Number];
		boolean Action_Complete = false;
		if (Action.action.length > Action_Number) {
			// does the action and if completed set Action_Complete to true
			switch (Current_Action) {
				case "drive":
					Action_Complete = DriveDistance(Action.goal[Action_Number][0]); // Drive distance in the x or y
																					// direction? What is theta?
					break;
				case "turn":
					Action_Complete = Turn(Action.goal[Action_Number][0]);
					break;
				case "arc":
					Action_Complete = Arc(Action.goal[Action_Number][0], Action.goal[Action_Number][1]);
					break;
				case "restart":
					Action_Number = 0;
					Rotation_Reference = imu.getAngle(imu.getYawAxis());
					setBaseLocation();
					break;
			}
			if (Action_Complete) {
				System.out.println("done action");
				Action_Number += 1;
				Rotation_Reference = imu.getAngle(imu.getYawAxis());
				double[] MotorSpeed = {0,0};
				accelMotors(MotorSpeed);
				setBaseLocation();
			}
			// updates the motors and actually sets the speed
			updateMotorsABCD();

			// Once the action is completed, move to next action
			if (Action_Complete) {
				Action_Number += 1;
				SmartDashboard.putNumber("ActionNumb", Action_Number);
				Rotation_Reference = imu.getAngle(imu.getYawAxis());
				setBaseLocation();
			}
		}
	}

	/**
	 * This function is called once each time the robot enters teleoperated mode.
	 */
	@Override
	public void teleopInit() {
		// lets motors either brake or roll(driver prefrence)
		leftRear.setIdleMode(IdleMode.kCoast);
		leftFront.setIdleMode(IdleMode.kCoast);
		rightRear.setIdleMode(IdleMode.kCoast);
		rightFront.setIdleMode(IdleMode.kCoast);

		// update dashboard action
		Current_Action = "TeleOp";

		// resets motor speeds
		Motors = new double[] { 0, 0 };

	}

	/** This function is called periodically during teleop */
	@Override
	public void teleopPeriodic() {
		// For Driving use one or the other
		// TwistRotate(Controller.getRawAxis(Constants.Joystick_twistAxis));
		// JoystickMovement(Controller.getRawAxis(Constants.Joystick_xAxis),
		// 		-Controller.getRawAxis(Constants.Joystick_yAxis));
		updateMotorsABCD();
		Launcher.shoot();
		// OR
		// m_drivetrain.arcadeDrive(-Controller.getRawAxis(Constants.Joystick_xAxis),
		// -Controller.getRawAxis(Constants.Joystick_yAxis), false);
	}

	/**
	 * Takes array of 4 values -1 to 1 and adjusts each motor value towards
	 * specified values
	 * 
	 * @param MotorGoal array of 4 doubles which the motors to should accelerate
	 *                  towards
	 */
	public void accelMotors(double MotorGoal[]) { // TODO This function is a point of failure, because it jerks
													// forward.
		// Sets both goal within max range
		double max = Math.max(Math.abs(MotorGoal[0]), Math.abs(MotorGoal[1]));
		if (max > Constants.SpeedLimit) {
			MotorGoal[0] *= Constants.SpeedLimit / max;
			MotorGoal[1] *= Constants.SpeedLimit / max;
		}
		// finds the largest diffrence between motor speeds
		double maxToGoal = Math.max(Math.abs(Motors[0] - MotorGoal[0]), Math.abs(Motors[1] - MotorGoal[1]));
		if (maxToGoal == 0) {
			return;
		}
		// variable to store the conversion rates to make sure all motors reach goal at
		// the same time
		// for example if motors are accelerating to 10 and 5 from 0 one will accelrate
		// half the speed to make the acceleration uniform
		double conversion;
		for (int i = 0; i < 2; i++) {
			conversion = (MotorGoal[i] - Motors[i]) / maxToGoal;
			if (Math.abs(Motors[i] - MotorGoal[i]) < Math.abs(Constants.Acceleration * conversion)) {
				// just sets to motor goal if speed is already close enough
				Motors[i] = MotorGoal[i];
			} else {
				Motors[i] += Constants.Acceleration * conversion;
			}
		}
	}

	/**
	 * Updates all motors based on values in the array Motors[]
	 * 
	 */
	public void updateMotorsABCD() {
		// Saferguard to prevent motors from going over the max speed of 1 and limit
		// based on constant
		double max = Math.max(Math.abs(Motors[0]), Math.abs(Motors[1]));
		if (max > Constants.SpeedLimit) { // if the max motor is more than limit proportionaly decreases all motor
											// speeds to within range
			for (int i = 0; i < 2; i++) {
				Motors[i] *= Constants.SpeedLimit / max;
			}
		}
		// update drive train motors
		m_drivetrain.tankDrive(Motors[0], Motors[1], false);
	}

	/**
	 * sets motors to rotate base on single twist axis
	 * 
	 * @param Axis of joystick twist
	 */
	public void TwistRotate(double Axis) {
		// Sets motors to turn in direction of twist
		double Motors[] = { Axis, -Axis, Axis, -Axis };
		accelMotors(Motors);
	}

	/**
	 * Calculates and sets power(-1 to 1) for rotational and arc turn movement
	 * 
	 * @param xAxis of joystick
	 * @param yAxis of joystick
	 */
	public void JoystickMovement(double xAxis, double yAxis) {
		// Commented Out V Built-In Drift Correction:

		if (xAxis != 0.0) { // Conditional if prevents indeterminate 0/0 = infinity or 0:
			int SignX = (int) ((Math.abs(xAxis)) / xAxis); // Determine the direction of the joystick
			double CorrectionX = ThresholdX * SignX * -1; // Change the "ThresholdX" threshold used for correction,
															// disable when Joystick isnt static?
			xAxis += CorrectionX; // Update the x,yAxis values.
			// Consider joystick dampening by f(x)?
			SmartDashboard.putNumber("CorrectedxAxis", xAxis); // For debugging, output x,yAxis
		}
		if (yAxis != 0.0) {
			int SignY = (int) ((Math.abs(yAxis)) / yAxis);
			double CorrectionY = ThresholdY * SignY * -1;
			yAxis += CorrectionY;
			SmartDashboard.putNumber("CorrectedyAxis", yAxis); // For debugging, output x,yAxis

		}
		// End of Drift Correction

		// Calculates speed and direction from the two axis of the Controller
		double direction = Math.atan2(yAxis, -xAxis) - Math.PI / 2;
		double speed = Math.sqrt(yAxis * yAxis + xAxis * xAxis);
		if (speed < 0.1) {
			speed = 0;
		} else if (speed < 0.5) {
			speed = 0.2;
		} else {
			speed = 5 / 2 * speed - 0.2;
		}
		speed *= (yAxis < 0) ? -1 : 1;
		if (Math.abs(direction) < Constants.LOCK_TO_AXIS_PI) {
			accelMotors(new double[] { speed, speed });
		} else if (Math.abs(direction + Math.PI / 2) < Constants.LOCK_TO_AXIS_PI) {
			ArcTurn(0, speed);
		} else if (Math.abs(direction - Math.PI / 2) < Constants.LOCK_TO_AXIS_PI) {
			ArcTurn(0, speed);
		} else {
			// math to get radius of turns
			double radius = 1 / (Math.tan(direction)) * Constants.TurningSensitivity;
			// math to decide if speed is positive or neagtive
			SmartDashboard.putNumber("Speed", speed);
			SmartDashboard.putNumber("Radius", radius);
			ArcTurn(radius, speed);
		}
	}

	/**
	 * Takes inputs and sets motors to move in specifed arc path
	 * 
	 * @param Radius of the turn from center of bot(0 results in pivot)
	 * @param Speed  of the turn (if radius 0 +ive arcs forward, -ive arcs backward
	 *               )
	 */
	public void ArcTurn(double Radius, double Speed) {
		double LeftSpeed, RightSpeed;
		if (Radius > 0) {
			// code for arcing right
			double TurnRatio = (Radius - Constants.RobotWidth / 2) / (Radius + Constants.RobotWidth / 2);
			LeftSpeed = Speed;
			RightSpeed = Speed * TurnRatio;
		} else {
			// code for arcing left
			double TurnRatio = (Radius + Constants.RobotWidth / 2) / (Radius - Constants.RobotWidth / 2);
			RightSpeed = Speed;
			LeftSpeed = Speed * TurnRatio;
		}
		// sets the motors
		double MotorSpeeds[] = { LeftSpeed, RightSpeed };
		accelMotors(MotorSpeeds);
	}

	/**
	 * Sets a base location to get relative displacement for future reference
	 * 
	 */
	public void setBaseLocation() {
		// resets encoders to calculate new distance travelled
		Encoder_LF.setPosition(0);
	}

	/**
	 * Returns x, y displacement of robot from base location
	 * (still not sure if this works)
	 */
	public double calculateMovement() {
		// conversion of rotations to distance
		double rotationToDistance = 2 * Math.PI * Constants.WheelRadius;

		// get rotations of motors (only uses first two because back two should be the
		// same assuming that there are no rotations)
		double MotorMovement = Encoder_LF.getPosition() * rotationToDistance / Constants.GEAR_RATIO;

		return MotorMovement;
	}

	/**
	 * Travels specified distance
	 * 
	 * @param distance to goal
	 * @return true or false if distance is completed
	 */
	public boolean DriveDistance(double distance) {

		// gets distances to goal
		double distance_travelled = calculateMovement();

		// variable if distance is reached
		boolean complete = false;

		if (Math.abs(distance - distance_travelled) <= 0.05) {
			// to avoid issues, if no movement just set to zero
			accelMotors(new double[] { 0, 0, 0, 0 });
			SmartDashboard.putNumber("STOP", distance - distance_travelled);
			complete = true;
		} else {
			// sets speed based on distance to the goal
			double Speed = distance - distance_travelled;
			double MotorSpeeds[] = { Speed, Speed };
			// adjusts the speed of each motor
			accelMotors(MotorSpeeds);
		}
		return complete;
	}

	/**
	 * turn to certain ammount
	 * 
	 * @param degrees goal (positive=clockwise)
	 * @return true or false if turn is completed
	 */
	public boolean Turn(double Degrees) {
		Degrees = Math.toRadians(Degrees);
		boolean complete = false;
		double goal = Rotation_Reference + Degrees;
		double Speed = goal - imu.getAngle(imu.getYawAxis());
		// if speed is low enough aka close enought to goal say turn is complete
		if (Math.abs(Speed) < 0.01) {
			complete = true;
		}
		// Sets motors to turn in direction of twist
		accelMotors(new double[] { Speed, -Speed });
		return complete;
	}

	/**
	 * 
	 * @param Degrees of turn (positive forward and negative backward)
	 * @param Radius  of turn (positive right and negative left)
	 * @return if arc is completed
	 */
	public boolean Arc(double Degrees, double Radius) {
		Degrees = Math.toRadians(Degrees);
		boolean complete = false;
		double goal = Rotation_Reference + (Degrees * ((Radius < 0) ? 1 : -1));
		double Speed = (goal - imu.getAngle(imu.getYawAxis())) * ((Degrees > 0) ? 1 : -1) * ((Radius < 0) ? 1 : -1);
		// if speed is low enough aka close enought to goal say turn is complete
		if (Math.abs(Speed) < 0.01) {
			complete = true;
		}
		// sets the motors to start turning
		ArcTurn(Radius, Speed);
		// returns weather turn is complete
		return complete;
	}
}