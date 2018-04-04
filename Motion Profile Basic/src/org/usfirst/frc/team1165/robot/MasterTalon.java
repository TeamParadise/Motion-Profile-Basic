package org.usfirst.frc.team1165.robot;

import static org.usfirst.frc.team1165.robot.RobotMap.DRIVE_STRAIGHT_SLOT;
import static org.usfirst.frc.team1165.robot.RobotMap.IN_TO_TICKS;
import static org.usfirst.frc.team1165.robot.RobotMap.TICKS_TO_IN;
import static org.usfirst.frc.team1165.robot.RobotMap.TIMEOUT;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MasterTalon extends WPI_TalonSRX
{
	public MasterTalon(int id, boolean phase, boolean isInverted, double[] pid)
	{
		super(id);
		
		setInverted(isInverted);

		configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, TIMEOUT);
		setSensorPhase(phase);
		
		configNeutralDeadband(0.1, TIMEOUT);
		
		configOpenloopRamp(0.1, TIMEOUT);
//		configClosedloopRamp(0.1, TIMEOUT);

		configVoltageCompSaturation(13, TIMEOUT);
		enableVoltageCompensation(true);
		
		configNominalOutputForward(0, TIMEOUT); // 0.1
		configNominalOutputReverse(0, TIMEOUT);
		configPeakOutputForward(0.75, TIMEOUT); // 0.6, 1
		configPeakOutputReverse(-0.75, TIMEOUT);
		
		configAllowableClosedloopError(DRIVE_STRAIGHT_SLOT, (int) IN_TO_TICKS, TIMEOUT);
		
		selectProfileSlot(DRIVE_STRAIGHT_SLOT, 0);
		config_kP(DRIVE_STRAIGHT_SLOT, pid[0], TIMEOUT);
		config_kI(DRIVE_STRAIGHT_SLOT, pid[1], TIMEOUT);
		config_kD(DRIVE_STRAIGHT_SLOT, pid[2], TIMEOUT);
		config_kF(DRIVE_STRAIGHT_SLOT, pid[3], TIMEOUT);

		// Motion profile stuff // 
		configMotionProfileTrajectoryPeriod(10, TIMEOUT);
		setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, TIMEOUT);
	}
	
	public double getPos() {
		return getSelectedSensorPosition(0) * TICKS_TO_IN;
	}
	
	public double getVel() {
		return getSelectedSensorVelocity(0) * 10 * TICKS_TO_IN;
	}
	
	public double getError() {
		return getClosedLoopError(0) * TICKS_TO_IN;
	}
	
	public void reset() {
		setSelectedSensorPosition(0, 0, TIMEOUT);
	}
	
	public boolean isReset() {
		return fuzzyEquals(getPos(), 0, 0.01);
	}
	
	public boolean onTarget(double target) {
		return fuzzyEquals(getPos(), target, 1);
	}
	
	private static boolean fuzzyEquals(double a, double b, double tolerance) {
		return Math.abs(Math.abs(a) - Math.abs(b)) < tolerance;		
	}
	
	public void report(String name)
	{
		SmartDashboard.putNumber(name + " Speed", getMotorOutputPercent());
		SmartDashboard.putNumber(name + " Position", getPos());
		SmartDashboard.putNumber(name + " Vel", getVel());
		SmartDashboard.putNumber(name + " Error", getError());
	}
}
