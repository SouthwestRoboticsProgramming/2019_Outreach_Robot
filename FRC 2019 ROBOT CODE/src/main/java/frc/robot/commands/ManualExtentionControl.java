package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ManualExtentionControl extends Command {
  public ManualExtentionControl() {
    requires(Robot.extentionSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double extentionControl = Robot.oi.extentionControl();
    boolean extentionLimitBypass = Robot.oi.wristExtentionlimitBypass();
    final double offset = .15;

    double extentionSpeed;
    if (extentionControl != 0) {
      extentionSpeed = (extentionControl * Robot.ShuffleBoard.extentionSpeed.getDouble(RobotMap.defaultExtentionSpeed));
    } else {
      extentionSpeed = 0;
    }

    if (extentionSpeed > 0) {
      extentionSpeed += offset;
    } else if (extentionSpeed < 0){
      extentionSpeed -= offset;
    }

    if (Robot.oi.isOutreachMode) {
      extentionSpeed *= Robot.ShuffleBoard.outreachModeExtentionSpeed.getDouble(RobotMap.defaultOutreachExtentionSpeed);
      // extentionMove = extentionMove * RobotMap.defaultExtentionSmooth;
    }

    //extention smoothing
    double extentionFinal = Robot.extentionSubsystem.getExtentionOutput() + ((extentionSpeed - Robot.extentionSubsystem.getExtentionOutput()) * Robot.ShuffleBoard.extentionSmooth.getDouble(RobotMap.defaultExtentionSmooth));
    // double extentionFinal = getExtentionOutput() + ((extentionMove - getExtentionOutput())* RobotMap.defaultExtentionSmooth);

    //send extentionFinal to ouput
    Robot.extentionSubsystem.driveMoter(extentionFinal, extentionLimitBypass);
    Robot.extentionSubsystem.extentionEncoderLowerReset();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.extentionSubsystem.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}