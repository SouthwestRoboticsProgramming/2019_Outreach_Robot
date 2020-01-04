package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ManualWristControl extends Command {
  public ManualWristControl() {
    requires(Robot.WristSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double wristSpeed = Robot.oi.wristControl();
    boolean wristLimitBypass = !Robot.oi.wristExtentionlimitBypass();

    double wristMove = ((-wristSpeed) / Robot.ShuffleBoard.wristSpeed.getDouble(RobotMap.defaultWristSpeed));

    if (Robot.oi.isOutreachMode) {
      wristMove = wristMove * Robot.ShuffleBoard.outreachModeWristSpeed.getDouble(RobotMap.defaultOutreachWristSpeed);
    }

    //Arm smoothing
      double wristFinal = Robot.WristSubsystem.getWristOutput() + ((wristMove - Robot.WristSubsystem.getWristOutput()) * Robot.ShuffleBoard.wristSmooth.getDouble(RobotMap.defaultWristSmooth));
    
    //run driveMoter
    Robot.WristSubsystem.driveMoter(wristFinal, wristLimitBypass);
      
  }

//   // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.WristSubsystem.stop();
   }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}