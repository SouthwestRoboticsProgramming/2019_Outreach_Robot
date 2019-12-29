package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Robot;
import frc.robot.Kinematics;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotMap;
import frc.robot.commands.OneJoyDrive;
import frc.robot.commands.TwoJoyDrive;
import frc.lib.Path.Path;
import frc.lib.Path.PathFollower;
import frc.lib.Path.Lookahead;
import frc.lib.Util.DriveSignal;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Twist2d;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.loops.Looper;


public class DriveSubsystem extends Subsystem {
	public boolean tunable = false;
	private boolean init = false;

	// paths
    private PathFollower mPathFollower;
	private Path mCurrentPath = null;
	private DriveControlState mDriveControlState;

	private WPI_TalonSRX leftMaster, leftSlave, rightMaster, rightSlave;

	private DifferentialDrive drive;
	Looper mEnabledLooper = new Looper();

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

		trackPosition(mEnabledLooper);

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

	/**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            setBrakeMode(true);
            System.out.println("switching to open loop");
            System.out.println(signal);
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }
    }

    public synchronized void trackPosition(ILooper in) {
        in.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {
				updatePathFollower(timestamp);                
            }

            @Override
            public void onStop(double timestamp) {
            }
        });
    }

	/**
     * Configure talons for velocity control
     */
    public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
        if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            setBrakeMode(true);
            System.out.println("switching to path following");
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
            // mLeftMaster.selectProfileSlot(kLowGearVelocityControlSlot, 0);
            // mRightMaster.selectProfileSlot(kLowGearVelocityControlSlot, 0);
            // mLeftMaster.configNeutralDeadband(0.0, 0);
            // mRightMaster.configNeutralDeadband(0.0, 0);
        }

        // mPeriodicIO.left_demand = signal.getLeft();
        // mPeriodicIO.right_demand = signal.getRight();
        // mPeriodicIO.left_feedforward = feedforward.getLeft();
        // mPeriodicIO.right_feedforward = feedforward.getRight();
    }

	/**
     * Configures the drivebase to drive a path. Used for autonomous driving
     *
     * @see Path
     */
    public synchronized void setWantDrivePath(Path path, boolean reversed) {
        if (mCurrentPath != path || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            RobotState.getInstance().resetDistanceDriven();
            mPathFollower = new PathFollower(path, reversed, new PathFollower.Parameters(
                    new Lookahead(Constants.kMinLookAhead, Constants.kMaxLookAhead, Constants.kMinLookAheadSpeed,
                            Constants.kMaxLookAheadSpeed),
                    Constants.kInertiaSteeringGain, Constants.kPathFollowingProfileKp,
                    Constants.kPathFollowingProfileKi, Constants.kPathFollowingProfileKv,
                    Constants.kPathFollowingProfileKffv, Constants.kPathFollowingProfileKffa,
                    Constants.kPathFollowingProfileKs, Constants.kPathFollowingMaxVel,
                    Constants.kPathFollowingMaxAccel, Constants.kPathFollowingGoalPosTolerance,
                    Constants.kPathFollowingGoalVelTolerance, Constants.kPathStopSteeringDistance));
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
            mCurrentPath = path;
        } else {
            setVelocity(new DriveSignal(0, 0), new DriveSignal(0, 0));
        }
    }

    public synchronized boolean isDoneWithPath() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            return mPathFollower.isFinished();
        } else {
            System.out.println("Robot is not in path following mode");
            return true;
        }
    }

    public synchronized void forceDoneWithPath() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            mPathFollower.forceFinish();
        } else {
            System.out.println("Robot is not in path following mode");
        }
    }

    private void updatePathFollower(double timestamp) {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            RobotState robot_state = RobotState.getInstance();
            Pose2d field_to_vehicle = robot_state.getLatestFieldToVehicle().getValue();
            Twist2d command = mPathFollower.update(timestamp, field_to_vehicle, robot_state.getDistanceDriven(),
                    robot_state.getPredictedVelocity().dx);
            if (!mPathFollower.isFinished()) {
                DriveSignal setpoint = Kinematics.inverseKinematics(command);
                setVelocity(setpoint, new DriveSignal(0, 0));
            } else {
                if (!mPathFollower.isForceFinished()) {
                    setVelocity(new DriveSignal(0, 0), new DriveSignal(0, 0));
                }
            }
        } else {
            DriverStation.reportError("drive is not in path following state", false);
        }
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

	public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        PATH_FOLLOWING, // velocity PID control
	}
	
	public enum DriveCurrentLimitState {
        UNTHROTTLED, THROTTLED
    }
}