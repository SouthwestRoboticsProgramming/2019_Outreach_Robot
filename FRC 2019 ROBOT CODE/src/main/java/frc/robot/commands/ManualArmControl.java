package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;


public class ManualArmControl extends Command {

  public ManualArmControl() {
    requires(Robot.armSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  double armSpeed = Robot.oi.armControl();
  boolean armLimitBypass = Robot.oi.armLimitBypass();
    double armMove = ((-armSpeed) * Robot.ShuffleBoard.armSpeed.getDouble(RobotMap.defaultArmSpeed));
    // double armMove = ((-armSpeed) * RobotMap.defaultArmSpeed);

    if (Robot.oi.isOutreachMode) {
      armMove = armMove * Robot.ShuffleBoard.outreachModeArmSpeed.getDouble(RobotMap.defaultOutreachArmSpeed);
      // armMove = armMove * RobotMap.defaultOutreachArmSpeed;
    } 

    //Arm smoothing
    double armFinal = Robot.armSubsystem.getArmOutput() + ((armMove - Robot.armSubsystem.getArmOutput()) * Robot.ShuffleBoard.armSmooth.getDouble(RobotMap.defaultArmSmooth));
    // double armFinal = getArmOutput() + ((armMove - getArmOutput()) * RobotMap.defaultArmSmooth);

    //Start driveMorer, and armBrake
    Robot.armSubsystem.driveMoter(armFinal, armLimitBypass);  
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.armSubsystem.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}