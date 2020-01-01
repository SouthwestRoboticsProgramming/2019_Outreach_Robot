package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.ManualDriveCommand;

public class DriveSubsystem extends Subsystem {

    public boolean tunable = false;

    private int frontLeftCANId = 3;
    private int frontRightCANId = 2;
    private int backLeftCANId = 0;
    private int backRightCANId = 1;

    private WPI_TalonSRX frontLeft, frontRight, backLeft, backRight;

    private double xAxisOutput = 0;
    private double yAxisOutput = 0;
    private double turnOutput = 0;

    public DriveSubsystem(boolean tunable) {
		this.tunable = tunable;
        frontLeft = new WPI_TalonSRX(frontLeftCANId);
        frontRight = new WPI_TalonSRX(frontRightCANId);
        backLeft = new WPI_TalonSRX(backLeftCANId);
        backRight = new WPI_TalonSRX(backRightCANId);

        frontLeft.setInverted(false);
        frontRight.setInverted(true);
        backLeft.setInverted(false);
        backRight.setInverted(true);
    }

    public void manualDrive(double xAxis, double yAxis, double leftTrigger, double rightTrigger) {
         // EDITABLES
            final double xAxisSmooth = .15;
            final double yAxisSmooth = .15;
            final double turnSmooth = .15;
 
         // INPUTS
            double turn = -leftTrigger + rightTrigger;
 
         // SMOOTHING
            double xSmoothed = 0;
            double ySmoothed = 0;
            double turnSmoothed = 0;
    
            xSmoothed = xAxisOutput + ((xAxis - xAxisOutput) * xAxisSmooth);
            ySmoothed = yAxisOutput + ((yAxis - yAxisOutput) * yAxisSmooth);
            turnSmoothed = turnOutput + ((turn - turnOutput) * turnSmooth);
    
         // OUTPUTS
            xAxisOutput = xSmoothed;
            yAxisOutput = ySmoothed;
            turnOutput = turnSmoothed;
    }

    public void driveMotors() {
        frontLeft.set(yAxisOutput - xAxisOutput + turnOutput);
        frontRight.set(yAxisOutput + xAxisOutput - turnOutput);
        backRight.set(yAxisOutput - xAxisOutput - turnOutput);
        backLeft.set(yAxisOutput + xAxisOutput + turnOutput);
    }

    public void stop() {
        frontLeft.stopMotor();
        frontRight.stopMotor();
        backRight.stopMotor();
        backLeft.stopMotor();
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new ManualDriveCommand());
    }

}
