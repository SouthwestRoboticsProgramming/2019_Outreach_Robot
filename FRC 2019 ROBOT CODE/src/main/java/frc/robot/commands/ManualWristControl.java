package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

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
    Robot.WristSubsystem.wristControl(Robot.oi.wristControl(), !Robot.oi.wristExtentionlimitBypass());
    Robot.WristSubsystem.wristEncoderUpperReset();
    Robot.WristSubsystem.wristEncoderLowerReset();

    // Robot.WristSubsystem.wristControl(Robot.oi.joystickArm.getZ(), Robot.oi.wristUpperLimitSwitch.get(), Robot.oi.wristLowerLimitSwitch.get(), Robot.oi.wristExtentionlimitBypass.get(), Robot.oi.armLimitBypassWristLock.get());
      
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