package org.usfirst.frc.team1165.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap
{
	public static final int JOYSTICK_PORT = 0;

	public static final int DRIVE_TALON_FRONT_LEFT = 23;
	public static final int DRIVE_TALON_REAR_LEFT = 21;
	public static final int DRIVE_TALON_FRONT_RIGHT = 20;
	public static final int DRIVE_TALON_REAR_RIGHT = 22;
	
	public static final double TICKS_TO_IN = Math.PI * 6 / 4096;
	public static final double IN_TO_TICKS = 4096 / (Math.PI * 6);
	
	public static final int TIMEOUT = 0;
	public static final int DRIVE_STRAIGHT_SLOT = 0;
	
	public static final double L = 23;
	public static final double W = 19;
	
	public static final int MAX_VELOCITY = 2400;
	
//	public static final double[] DRIVE_STRAIGHT_LEFT_PID = { 0.02, 0, 0, 0 };

	
	public static final double[] DRIVE_STRAIGHT_LEFT_PID = { 0, 0, 0, 0.34678 };
	public static final double[] DRIVE_STRAIGHT_RIGHT_PID = { 0, 0, 0, 0.32476 };

//	public static final double[] DRIVE_STRAIGHT_LEFT_PID = { 0.32, 0, 0, 0 };
//	public static final double[] DRIVE_STRAIGHT_RIGHT_PID = { 0.25, 0, 0, 0 };
}
