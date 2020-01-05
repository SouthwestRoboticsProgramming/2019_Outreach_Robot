package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.commands.BallShooterToggle;

public class BallShooter extends Subsystem {

    public boolean motorOn = false;
    public boolean ramp = false;

    private WPI_TalonSRX motor;

    public BallShooter() {
        motor = new WPI_TalonSRX(4);
        motor.setInverted(false);
    }

    public void toggle() {
        // motor.stopMotor();
    }

    // public double getShooterSpeed() {
    //     return SmartDashboard.getNumber("shooterSpeed", 1);
    // }

    public void start() {
        if (!ramp && !motorOn) {
            new Thread(new Runnable() {
                public void run() {
                    ramp = true;
                    System.out.println("Start started");
                    Robot.DriveSubsystem.stop();
                    while (motor.get() < Robot.ShuffleBoard.shooterSpeed.getDouble(1)) {
                    motor.set(motor.get() + .01);
                    Timer.delay(.02);
                    }
                    motor.set(Robot.ShuffleBoard.shooterSpeed.getDouble(1));
                    ramp = false;
                    motorOn = true;
                }
            }).start();
            
        }
        
    }
  
    public void stop() {
        if (!ramp && motorOn) {
            new Thread(new Runnable() {
                public void run() {
                    ramp = true;
                    System.out.println("Stop started");
                    Robot.DriveSubsystem.stop();
                    while (motor.get() > 0) {
                    motor.set(motor.get() - .01);
                    Timer.delay(.02);
                    }
                    motor.set(0);
                    ramp = false;
                    motorOn = false;
                }
            }).start();
        }
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new BallShooterToggle());
    }

}
