package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.OneJoyDrive;
import frc.robot.commands.TwoJoyDrive;


public class DriveSubsystem extends Subsystem {
	public boolean tunable = false;
	private boolean init = false;

	private WPI_TalonSRX leftMaster, leftSlave, rightMaster, rightSlave;

	private DifferentialDrive drive;

	public DriveSubsystem(boolean tunable) {
		this.tunable = tunable;
		//setup motors
		leftMaster = new WPI_TalonSRX(RobotMap.leftMasterPort);
		leftSlave = new WPI_TalonSRX(RobotMap.leftSlavePort);
		rightMaster = new WPI_TalonSRX(RobotMap.rightMasterPort);
		rightSlave = new WPI_TalonSRX(RobotMap.rightSlavePort);

		drive = new DifferentialDrive(leftMaster, rightMaster);
		
		// RESET TALONS
		leftMaster.configFactoryDefault();
		leftSlave.configFactoryDefault();
		rightMaster.configFactoryDefault();
		rightSlave.configFactoryDefault();
		
		// LEFT MASTER
		leftMaster.setInverted(true);
		leftSlave.setInverted(true);
		leftMaster.setSensorPhase(false);
		leftMaster.setNeutralMode(NeutralMode.Brake);
		
		// FOLLOW
		leftSlave.follow(leftMaster);
		
		// RIGHT MASTER
		rightMaster.setInverted(false);
		rightSlave.setInverted(false);
		rightMaster.setSensorPhase(false);	
		rightMaster.setNeutralMode(NeutralMode.Brake);

		// FOLLOW
		rightSlave.follow(rightMaster);
	}

	public void init() {
		leftMaster.setSelectedSensorPosition(0);
		rightMaster.setSelectedSensorPosition(0);

		if (Robot.oi.controller.getDefaultControllerSet().equals(Robot.oi.threeJoy)) {
			setDefaultCommand(new TwoJoyDrive());
		} else {
			setDefaultCommand(new OneJoyDrive());
		}

		init = true;
	}

	public double getLeftOutput() {
		return leftMaster.get();
	}

	public double getRightOutput() {
		return rightMaster.get();
	}

	//GET ENCODOR OUTPUT
	public double getLeftDriveEncoder() {
		return leftMaster.getSelectedSensorPosition();
	}

	public double getRightDriveEncoder() {
		return rightMaster.getSelectedSensorPosition();
	}

	public double getLeftDriveFeet() {
		return leftMaster.getSelectedSensorPosition() / 1024 * Math.PI * 6;
	}

	public double getRightDriveFeet() {
		return rightMaster.getSelectedSensorPosition() / 1024 * Math.PI * 6;
	}

	public void setBrakeMode(boolean mode) {
		NeutralMode neutralMode;
		if (mode) {
			neutralMode = NeutralMode.Brake;
		} else {
			neutralMode = NeutralMode.Coast;
		}
		leftMaster.setNeutralMode(neutralMode);
		leftSlave.setNeutralMode(neutralMode);
		rightMaster.setNeutralMode(neutralMode);
		rightSlave.setNeutralMode(neutralMode);
	}

	public void lineFollow(double LimelightXValue, double joystickY) {

		double lineOffSet = (LimelightXValue / 15 * .25);
		
		// if (targetArea >= 8 || targetArea == 0) {
		// 	// KEEP LINE OFFSET THE SAME
		// } else if (targetArea < 2) {
		// 	if (XValue >= 0) {
		// 		lineOffSet = lineOffSet - .1;
		// 	} else if (XValue <= 0) {
		// 		lineOffSet = lineOffSet + .1;
		// 	}
		// }

		double leftOutput = joystickY - lineOffSet;
		double rightOutput = joystickY + lineOffSet;
		driveMotors(leftOutput, rightOutput);

	}

	public void driveLeftMotor(double left) {
		driveMotors(left, getRightOutput());
	}

	public void driveRightMotor(double right) {
		driveMotors(getLeftOutput(), right);
	}

	public void driveMotors(double left, double right) {
		if (init) {
			drive.tankDrive(left, right);
			Robot.ShuffleBoard.driveLeftOutput.setDouble(leftMaster.get());
			Robot.ShuffleBoard.driveRightOutput.setDouble(rightMaster.get());
		}
	}

	public void stop() {
		drive.tankDrive(0, 0);		
	}
	
	@Override
	public void initDefaultCommand() {
	}

}