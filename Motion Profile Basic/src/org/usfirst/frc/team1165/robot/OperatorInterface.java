package org.usfirst.frc.team1165.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OperatorInterface
{
	private static final OperatorInterface mInstance = new OperatorInterface();

	public static OperatorInterface getInstance() {
		return mInstance;
	}
	
	private Joystick mJoy = new Joystick(0);

	// ----- Joystick Values ----- //
	
	public double getX()
	{
		return dampen(mJoy.getX());
	}

	public double getY()
	{
		return dampen(-mJoy.getY());
	}

	public double getTwist()
	{
		return dampen(mJoy.getTwist());
	}

	public double getThrottle()
	{
		return (1 - mJoy.getThrottle()) / 2;
	}

	public boolean getButton(int button) {
		return mJoy.getRawButton(button);
	}
	
	// ----- Input Transform ----- //

	public static double dampen(double input)
	{
		return dampen(input, 3);
	}
	
	public static double dampen(double input, double dampener) {
		return Math.copySign(Math.pow(input, dampener), input);
	}

	public static double constrain(double input, double high, double low) {
		return Math.max(low, Math.min(input, high));
	}
	
	public static double deadband(double input, double deadband) {
		return Math.abs(input) > deadband? input: 0;
	}

	public void report()
	{
//		SmartDashboard.putNumber("OI X", getX());
//		SmartDashboard.putNumber("OI Y", getY());
//		SmartDashboard.putNumber("OI Twist", getTwist());
//		SmartDashboard.putNumber("OI Throttle", getThrottle());
	}
}