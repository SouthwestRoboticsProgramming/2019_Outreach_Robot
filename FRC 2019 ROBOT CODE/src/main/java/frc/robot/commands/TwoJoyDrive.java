
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class TwoJoyDrive extends Command {
  public TwoJoyDrive() {
    requires(Robot.driveSubsystem);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    // if (Robot.oi.LimelightControl()) {
    //   Robot.driveSubsystem.lineFollow(Robot.Limelight.limelightX(), Robot.oi.leftDrive());
    // } else {
    //   Robot.driveSubsystem.manualDrive(Robot.oi.leftDrive(), Robot.oi.rightDrive(), Robot.oi.rightControl(), Robot.oi.LimelightControl(), Robot.oi.rightDrive(), -Robot.oi.leftDrive());
    // }

    double leftPower = Robot.oi.leftDrive();
    double rightPower = Robot.oi.rightDrive();
    boolean rightControl = Robot.oi.rightControl();
    double rightTurn = leftPower;
 
		// left drive smooth
			double leftSpeed = Robot.driveSubsystem.getLeftOutput() + ((leftPower - Robot.driveSubsystem.getLeftOutput()) * Robot.ShuffleBoard.driveSmooth.getDouble(RobotMap.defaultDriveSmooth));

		//right drive smooth
			double rightSpeed = Robot.driveSubsystem.getRightOutput() + ((rightPower - Robot.driveSubsystem.getRightOutput()) * Robot.ShuffleBoard.driveSmooth.getDouble(RobotMap.defaultDriveSmooth));

		//drive speed
			double leftFinal = leftSpeed * Robot.ShuffleBoard.driveSpeed.getDouble(RobotMap.defaultDriveSpeed);
			double rightFinal = rightSpeed * Robot.ShuffleBoard.driveSpeed.getDouble(RobotMap.defaultDriveSpeed);

			if (Robot.oi.isOutreachMode) {
				leftFinal = leftFinal * Robot.ShuffleBoard.outreachModeDriveSpeed.getDouble(RobotMap.defaultOutreachDriveSpeed);
				rightFinal = rightFinal * Robot.ShuffleBoard.outreachModeDriveSpeed.getDouble(RobotMap.defaultOutreachDriveSpeed);
			}

		// right control
			double leftOutput;
			double rightOutput;
			if (rightControl) {
				// MAKE WEELS ALLWAYS SLOW, AND NOT ACCELERATE
				if (rightTurn > 0) {
					leftOutput = rightFinal; 
					rightOutput = (rightFinal + (rightTurn * Robot.ShuffleBoard.driveRightControlLeftJoyEffectiveness.getDouble(RobotMap.defaultRightControlLeftEffectiveness)));
				} else {
					leftOutput = (rightFinal - (rightTurn * Robot.ShuffleBoard.driveRightControlLeftJoyEffectiveness.getDouble(RobotMap.defaultRightControlLeftEffectiveness)));
					rightOutput = rightFinal;
				}
				
			} else {
				leftOutput = leftFinal;
				rightOutput = rightFinal;
			}

			Robot.driveSubsystem.driveMotors(leftOutput, rightOutput);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    Robot.driveSubsystem.stop();
  }

  @Override
  protected void interrupted() {
    end();
  }
}