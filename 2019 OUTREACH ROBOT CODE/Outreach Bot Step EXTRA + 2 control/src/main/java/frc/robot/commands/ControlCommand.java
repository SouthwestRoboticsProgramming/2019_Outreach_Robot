package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class ControlCommand extends Command {
  public ControlCommand() {
    requires(Robot.Control);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (Robot.OI.coach.getX(Hand.kLeft) == 0) {
      Robot.OI.joystick = 1;
    } else {
      Robot.OI.joystick = 2;
    }
    SmartDashboard.putNumber("outreachSpeedMultiplyer", .5);
    SmartDashboard.putBoolean("outreachMode", false);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // OUTREACHMODE ACTIVE
      Robot.OI.outreachMode = SmartDashboard.getBoolean("outreachMode", false);
      Robot.OI.outreachSpeed = SmartDashboard.getNumber("outreachSpeedMultiplyer", .5);

    // SWITCH CONTROL
    if (Robot.OI.coach.getBButton()) {
      Robot.OI.joystick = 0;
      
      Robot.Control.rumbleController(RumbleType.kLeftRumble, false);
      Robot.Control.rumbleCoach(RumbleType.kLeftRumble, false);

    } else if (Robot.OI.coach.getBumper(Hand.kLeft)) {
      if (Robot.OI.joystick == 0) {

        Robot.Control.rumbleController(RumbleType.kRightRumble, false);
        Robot.Control.rumbleCoach(RumbleType.kRightRumble, true);
        Robot.Control.rumbleCoach(RumbleType.kRightRumble, false);
        
        Robot.OI.joystick = 1;

      } else if (Robot.OI.joystick == 2) {

        Robot.Control.rumbleController(RumbleType.kRightRumble, false);
        Robot.Control.rumbleCoach(RumbleType.kLeftRumble, true);
        Robot.Control.rumbleCoach(RumbleType.kRightRumble, true);
        Robot.Control.rumbleCoach(RumbleType.kRightRumble, false);

        Robot.OI.joystick = 1;

      }
      
    } else if (Robot.OI.coach.getBumper(Hand.kRight)) {
      if (Robot.OI.joystick == 0) {

        Robot.Control.rumbleCoach(RumbleType.kRightRumble, false);

        Robot.OI.joystick = 2;

      } else if (Robot.OI.joystick == 1) {

        Robot.Control.rumbleController(RumbleType.kLeftRumble, false);
        Robot.Control.rumbleCoach(RumbleType.kRightRumble, false);

        Robot.OI.joystick = 2;

      }
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
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
