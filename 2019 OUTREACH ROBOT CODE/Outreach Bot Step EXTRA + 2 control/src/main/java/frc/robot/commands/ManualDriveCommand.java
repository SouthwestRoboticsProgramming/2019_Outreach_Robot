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
   
    if (Robot.OI.joystick == 1) {
      double axAxis = Robot.OI.controller.getX(Hand.kLeft);
      double ayAxis = Robot.OI.controller.getY(Hand.kLeft);
      double aleftTrigger = Robot.OI.controller.getTriggerAxis(Hand.kLeft);
      double arightTrigger = Robot.OI.controller.getTriggerAxis(Hand.kRight);
      Robot.DriveSubsystem.manualDrive(axAxis, ayAxis, aleftTrigger, arightTrigger);
      Robot.DriveSubsystem.driveMotors();

      // Robot.DriveSubsystem.addAngle(Robot.OI.controller, 90, 90);
      // Robot.DriveSubsystem.addAngle(Robot.OI.controller, 180, 180);
      // Robot.DriveSubsystem.addAngle(Robot.OI.controller, -90, 270);
      // Robot.DriveSubsystem.addAngle(Robot.OI.controller, 360, 0);
    } else if (Robot.OI.joystick == 2) {
      double axAxis = Robot.OI.coach.getX(Hand.kLeft);
      double ayAxis = Robot.OI.coach.getY(Hand.kLeft);
      double aleftTrigger = Robot.OI.coach.getTriggerAxis(Hand.kLeft);
      double arightTrigger = Robot.OI.coach.getTriggerAxis(Hand.kRight);
      Robot.DriveSubsystem.manualDrive(axAxis, ayAxis, aleftTrigger, arightTrigger);
      Robot.DriveSubsystem.driveMotors();

      Robot.DriveSubsystem.addAngle(Robot.OI.coach, 90, 90);
      Robot.DriveSubsystem.addAngle(Robot.OI.coach, 180, 180);
      Robot.DriveSubsystem.addAngle(Robot.OI.coach, -90, 270);
      Robot.DriveSubsystem.addAngle(Robot.OI.coach, 360, 0);
    } else {
      Robot.DriveSubsystem.stop();
    }
    
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
