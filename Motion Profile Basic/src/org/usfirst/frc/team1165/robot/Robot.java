/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1165.robot;

import static org.usfirst.frc.team1165.robot.GeneratedMotionProfile.Points;
import static org.usfirst.frc.team1165.robot.RobotMap.*;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot
{
	private MasterTalon mFrontLeft = new MasterTalon(DRIVE_TALON_FRONT_LEFT, false, false, DRIVE_STRAIGHT_LEFT_PID);
	private MasterTalon mFrontRight = new MasterTalon(DRIVE_TALON_FRONT_RIGHT, false, false, DRIVE_STRAIGHT_RIGHT_PID);

	private WPI_TalonSRX mRearLeft = new WPI_TalonSRX(DRIVE_TALON_REAR_LEFT);
	private WPI_TalonSRX mRearRight = new WPI_TalonSRX(DRIVE_TALON_REAR_RIGHT);

	private DifferentialDrive mDrive = new DifferentialDrive(mFrontLeft, mFrontRight);
	
	private MotionProfileStatus mLeftStatus = new MotionProfileStatus();
	private MotionProfileStatus mRightStatus = new MotionProfileStatus();

	private OperatorInterface mOI = OperatorInterface.getInstance();


	private Notifier mNotifier = new Notifier(() -> {
		mFrontLeft.processMotionProfileBuffer();
		mFrontRight.processMotionProfileBuffer();
	});
	
	private boolean hasProfileEnded = false;
	
	@Override
	public void robotInit()
	{
		System.out.println("Robot Init");

		mRearLeft.follow(mFrontLeft);
		mRearRight.follow(mFrontRight);
		
		mRearLeft.setInverted(false);
		mRearRight.setInverted(false);
		
		mFrontLeft.changeMotionControlFramePeriod(5);
		mFrontRight.changeMotionControlFramePeriod(5);
		mNotifier.startPeriodic(0.005);
	}
	
	@Override
	public void disabledInit()
	{
		System.out.println("Disabled Init");

		mFrontLeft.clearMotionProfileTrajectories();
		mFrontRight.clearMotionProfileTrajectories();

		fill(mFrontLeft, Points, Points.length, 1);
		fill(mFrontRight, Points, Points.length, -1);

		mFrontLeft.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable.value);
		mFrontRight.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable.value);

		hasProfileEnded = false;
	}
	
	@Override
	public void robotPeriodic() {
		mFrontRight.report("Right");
		mFrontLeft.report("Left");
	}
	
	@Override
	public void teleopPeriodic() {
//		mFrontLeft.set(ControlMode.PercentOutput, mOI.getY());
//		mFrontRight.set(ControlMode.PercentOutput, mOI.getY());
		
		mDrive.arcadeDrive(mOI.getY(), mOI.getTwist());
	}
	
	@Override
	public void autonomousPeriodic()
	{
		mFrontLeft.getMotionProfileStatus(mLeftStatus);
		mFrontRight.getMotionProfileStatus(mRightStatus);
		
//		System.out.println(hasProfileEnded);
		
		if(hasProfileEnded) {
			mFrontLeft.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable.value);
			mFrontRight.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable.value);
		} else {
			mFrontLeft.set(ControlMode.MotionProfile, SetValueMotionProfile.Enable.value);
			mFrontRight.set(ControlMode.MotionProfile, SetValueMotionProfile.Enable.value);
			
			if(mLeftStatus.isLast && mRightStatus.isLast) hasProfileEnded = true;
			
//			System.out.println(mStatus.topBufferCnt + "\t" + mStatus.btmBufferCnt);
//			System.out.println(mFrontRight.getActiveTrajectoryPosition() + "\t" + mFrontRight.getActiveTrajectoryVelocity());
//			System.out.println(mFrontRight.getPos() + "\t" + -mFrontRight.getActiveTrajectoryPosition() * RobotMap.TICKS_TO_IN);

			System.out.println(-mFrontLeft.getPos() + "\t" + mFrontRight.getPos());
		}
	}
	

	public static void fill(MasterTalon talon, double[][] points, double length, double flip)
	{
		talon.configMotionProfileTrajectoryPeriod(0, TIMEOUT);

		TrajectoryPoint point = new TrajectoryPoint();

		point.headingDeg = 0; // unused
		point.profileSlotSelect0 = 0;
		point.profileSlotSelect1 = 0; // unused
		point.timeDur = TrajectoryDuration.Trajectory_Duration_10ms;

		for (int i = 0; i < length; i++)
		{
			point.position = points[i][0] * IN_TO_TICKS * flip;
			point.velocity = points[i][1] * IN_TO_TICKS * flip / 10;

			point.zeroPos = false;
			if (i == 0) point.zeroPos = true;
			
			point.isLastPoint = false;
			if ((i + 1) == length)
				point.isLastPoint = true;
			
			talon.pushMotionProfileTrajectory(point);
		}
	}
}
