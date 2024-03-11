// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class which contains all constants in one location
 */
public final class Constants {
	// ports and IDs
	public static final int Joystick_Port = 0;
	public static final int Arm_Port = 5;
	public static final int Intake_Port = 6;
	public static final int LEFT_FRONT_ID = 3;
	public static final int RIGHT_FRONT_ID = 2;
	public static final int LEFT_BACK_ID = 1;
	public static final int RIGHT_BACK_ID = 4;
	public static final int TEST_MOTOR_ID = 5;

	// Controller axis and buttons
	public static final int Joystick_xAxis = 0;
	public static final int Joystick_yAxis = 1;
	public static final int Joystick_ClimbAxis = 3;
	public static final int Joystick_twistAxis = 6;
	public static final int Launcher_Button = 1;
	public static final int Amp_Button = 2;
	public static final int Claw_In_Button = 3;
	public static final int Claw_Out_Button = 4;
	public static final int Intake_Button = 5;
	public static final int Feeder_Button = 6;

	// dimensions in M
	public static final double RobotWidth = 0.62;
	public static final double WheelRadius = 0.2;
	public static final double GEAR_RATIO = 28.5714285714; // ratio of 1:x on motor gears

	// movement presets
	public static double SpeedLimit = 0.1; // multiplier for overall speed (less than 1)
	public static double Acceleration = 0.01;// how much to accelerate
	public static final double DistanceAdjust = 1;// multiplier of distance output using encoders
	public static final double TurningSensitivity = 0.2; // how much to turn in arcRotation Teleop Mod
	public static final double LOCK_TO_AXIS_PI = Math.PI / 18 * 2; // 20 degree, while in range, to 0 move straight

	// PID
	public static final double MOTORKP = 0;
	public static final double MOTORKI = 0;
	public static final double MOTORKD = 0;

	// EVERYBOT CONSTANTS
	static final int DRIVE_CURRENT_LIMIT_A = 80;
	static final int FEEDER_CURRENT_LIMIT_A = 80;
	static final double FEEDER_OUT_SPEED = 1.0;
	static final double FEEDER_IN_SPEED = -.4;
	static final double FEEDER_AMP_SPEED = .4;
	static final int LAUNCHER_CURRENT_LIMIT_A = 80;
	static final double LAUNCHER_SPEED = 1;
	
	static final double CLAW_OUTPUT_POWER = .5;
	static final double CLAW_STALL_POWER = .1;
	static final double CLIMER_OUTPUT_POWER = 1;

}