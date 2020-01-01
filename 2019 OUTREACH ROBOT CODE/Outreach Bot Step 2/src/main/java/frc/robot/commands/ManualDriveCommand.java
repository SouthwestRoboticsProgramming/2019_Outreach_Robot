package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ManualDriveCommand extends Command {
  public ManualDriveCommand() {
    requires(Robot.DriveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double xAxis = Robot.OI.controller.getX(Hand.kLeft);
    double yAxis = Robot.OI.controller.getY(Hand.kLeft);
    double leftTrigger = Robot.OI.controller.getTriggerAxis(Hand.kLeft);
    double rightTrigger = Robot.OI.controller.getTriggerAxis(Hand.kRight);
    Robot.DriveSubsystem.manualDrive(xAxis, yAxis, leftTrigger, rightTrigger);

    Robot.DriveSubsystem.driveMotors();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.DriveSubsystem.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.DriveSubsystem.stop();
  }
}
