
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.lib.CheesyDrive;
import frc.robot.Robot;

public class OneJoyDrive extends Command {

  public static final String NAME = "gamepaddrivecommand";

    private CheesyDrive cheesyDrive = new CheesyDrive();

    private double prevPowLeft = 0;
    private double prevPowRight = 0;
    private double prevRotation = 0;

    public static final double SET_SPEED_DIFF_MAX = 0.08;


  public OneJoyDrive() {
    requires(Robot.driveSubsystem);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    double leftPow = Robot.oi.oneDrive();
    double rightPow = leftPow;
    double rotation = Robot.oi.oneTurn() * .55;

    if (!Robot.ShuffleBoard.cheezyDrive.getBoolean(true)) { // Arcade Drive
      leftPow = limitAcceleration(leftPow, prevPowLeft);
      rightPow = limitAcceleration(rightPow, prevPowRight);
      rotation = limitAcceleration(rotation, prevRotation);

      prevPowLeft = leftPow;
      prevPowRight = rightPow;
      prevRotation = rotation;
    } else { // Cheesy Drive
      boolean quickTurn = Robot.oi.oneQuickTurn();
      var signal = cheesyDrive.cheesyDrive(leftPow, rotation, quickTurn, false);
      leftPow = signal.getLeft();
      rightPow = signal.getRight();

      // CheezyDrive takes care of rotation so set to 0 to keep or code from adjusting
      rotation = 0;
    }
    Robot.driveSubsystem.driveMotors(leftPow + rotation, rightPow - rotation);
    
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }

  private double limitAcceleration(double setPow, double prevPow) {
    if (Math.abs(setPow - prevPow) > SET_SPEED_DIFF_MAX) {
        if (setPow > prevPow) {
            return prevPow + SET_SPEED_DIFF_MAX;
        } else if (setPow < prevPow) {
            return prevPow - SET_SPEED_DIFF_MAX;
        } else {
            return prevPow;
        }
    } else {
        return prevPow;
    }
}
}