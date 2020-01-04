package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class PresentPositions extends Command {
  public double armPresetPosition;
  public double extentionPresetPosition;
  public double wristPresetPosition;
  public boolean presetPostitonGoto;
  public double limelightPipeline;
  public int dateStamp;
  public int dateSeconds;
  public int timer;
  public int position;
  

  public PresentPositions(int positionID) {
    position = positionID;
    requires(Robot.armSubsystem);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (position == 1) {
      System.out.println("position 1");
      armPresetPosition = Robot.ShuffleBoard.armPickUpHatchPosition.getDouble(50);
      extentionPresetPosition = Robot.ShuffleBoard.extentionPickUpHatchPosition.getDouble(50);
      wristPresetPosition = Robot.ShuffleBoard.wristPickUpHatchPosition.getDouble(50);
      limelightPipeline = Robot.ShuffleBoard.pipelinePickUpHatchPosition.getDouble(50);
    } else if (position == 2) {
      System.out.println("position 2");
      armPresetPosition = Robot.ShuffleBoard.armPickUpBallPosition.getDouble(50);
      extentionPresetPosition = Robot.ShuffleBoard.extentionPickUpBallPosition.getDouble(50);
      wristPresetPosition = Robot.ShuffleBoard.wristPickUpBallPosition.getDouble(50);
      limelightPipeline = Robot.ShuffleBoard.pipelinePickUpBallPosition.getDouble(50);
    } else if (position == 3) {
      System.out.println("position 3");
      armPresetPosition = Robot.ShuffleBoard.armPlaceHatchPosition.getDouble(50);
      extentionPresetPosition = Robot.ShuffleBoard.wristPlaceHatchPosition.getDouble(50);
      wristPresetPosition = Robot.ShuffleBoard.wristPlaceHatchPosition.getDouble(50);
      limelightPipeline = Robot.ShuffleBoard.pipelinePlaceHatchPosition.getDouble(50);
    } else if (position == 4) {
      System.out.println("position 4");
      armPresetPosition = Robot.ShuffleBoard.armpickUpBallOffFloorPosition.getDouble(50);
      extentionPresetPosition = Robot.ShuffleBoard.extentionPickUpBallOffFloorPosition.getDouble(50);
      wristPresetPosition = Robot.ShuffleBoard.wristPickUpBallOffFloorPosition.getDouble(50);
      limelightPipeline = Robot.ShuffleBoard.pipelinePickUpBallOffFloorPosition.getDouble(50);
    } else if (position == 5) {
      System.out.println("position 5");
      armPresetPosition = Robot.ShuffleBoard.armPlaceBallInRocketPosition.getDouble(50);
      extentionPresetPosition = Robot.ShuffleBoard.extentionPlaceBallInRocketPosition.getDouble(50);
      wristPresetPosition = Robot.ShuffleBoard.wristPlaceBallInRocketPosition.getDouble(50);
      limelightPipeline = Robot.ShuffleBoard.pipelinePlaceBallInRocketPosition.getDouble(50);
    } else if (position == 6) {
      System.out.println("position 6");
      armPresetPosition = Robot.ShuffleBoard.armPlaceBallInCargoShipPosition.getDouble(50);
      extentionPresetPosition = Robot.ShuffleBoard.extentionPlaceBallInCargoShipPosition.getDouble(50);
      wristPresetPosition = Robot.ShuffleBoard.wristPlaceBallInCargoShipPosition.getDouble(50);
      limelightPipeline = Robot.ShuffleBoard.pipelinePlaceBallInCargoShipPosition.getDouble(50);
    } else if (position == 7) {
      System.out.println("position 7");
      armPresetPosition = Robot.ShuffleBoard.armVerticlePosition.getDouble(50);
      extentionPresetPosition = Robot.ShuffleBoard.extentionVerticlePosition.getDouble(50);
      wristPresetPosition = Robot.ShuffleBoard.wristVerticlePosition.getDouble(50);
      limelightPipeline = Robot.ShuffleBoard.pipelineVerticlePosition.getDouble(50);
    }

    dateStamp = (int)System.currentTimeMillis();
    presetPostitonGoto = false;
    Robot.Limelight.limelightSetPipeline((int)limelightPipeline);

    System.out.println("arm setAngle = " + armPresetPosition / 100);
    Robot.armSubsystem.armPIDToAngle(armPresetPosition / 100);

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // boolean armGoto = Robot.armSubsystem.armSetAngle(armPresetPosition / 100);
    // boolean extentionGoto = Robot.extentionSubsystem.extentionSetPosition(extentionPresetPosition / 100);
    
    // dateSeconds = (int)System.currentTimeMillis();
    // if (!armGoto && !extentionGoto) {
    //   presetPostitonGoto = true;
    // }

    boolean armGoto = Robot.armSubsystem.getGoingToPosition();
    // boolean extentionGoto = Robot.extentionSubsystem.getGoingToPosition();
    // boolean wristGoto = Robot.extentionSubsystem.getGoingToPosition();

    dateSeconds = (int)System.currentTimeMillis();
    if (!armGoto) {
      presetPostitonGoto = true;
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return presetPostitonGoto;
    // return (presetPostitonGoto || dateSeconds >= (dateStamp + (Robot.ShuffleBoard.presetPositionsTimeoutSeconds.getDouble(RobotMap.defaultPresetPositionsTimeout) * 1000)));

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