package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import com.analog.adis16448.frc.ADIS16448_IMU;
import frc.robot.sensors.ShuffleBoard;
import frc.robot.subsystems.BallShooter;
import frc.robot.subsystems.Control;
import frc.robot.subsystems.DriveSubsystem;

public class Robot extends TimedRobot {

  // Subsystem Instantiations
  public static DriveSubsystem DriveSubsystem = new DriveSubsystem(true);
  public static Control Control = new Control();
  public static BallShooter BallShooter = new BallShooter();
  public static ADIS16448_IMU ADIS16448_IMU = new ADIS16448_IMU();
  public static ShuffleBoard ShuffleBoard = new ShuffleBoard();
  public static OI OI;

  public boolean stop = false;

  @Override
  public void robotInit() {
    OI = new OI();
    DriveSubsystem.resetSetAngle();
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    
  }

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    
  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void testPeriodic() {
  }
}
