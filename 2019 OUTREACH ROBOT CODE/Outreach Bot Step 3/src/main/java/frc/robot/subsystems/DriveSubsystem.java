package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
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

    private double setAngle = 0;

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
            
        // INPUTS
            double turn = -leftTrigger + rightTrigger;
 
        // SMOOTHING
            double xSmoothed = 0;
            double ySmoothed = 0;
    
            xSmoothed = xAxisOutput + ((xAxis - xAxisOutput) * xAxisSmooth);
 
            ySmoothed = yAxisOutput + ((yAxis - yAxisOutput) * yAxisSmooth);

            setAngle = setAngle + (turn * (3 - (2 * ySmoothed)));
           
        // OUTPUTS
            xAxisOutput = xSmoothed;
            yAxisOutput = ySmoothed;
    }

    public double getAngleZ() {
        return Robot.ADIS16448_IMU.getAngleZ();
    }
    
    public void resetSetAngle() {
        setAngle = getAngleZ();
    }

    public void driveMotors() {
        // EDITABLES
            final double autoCorrectAngle = 90; // THE ANGLE THAT THE AUTO CORRECT WILL CAUSE THE MOTORS TO SPIN AT 100%
            final double maxSpinSpeed = .7; //THE MAX SPEED THE MOTORS WILL SPIN WHEN TURNING

        // INPUTES
            double xAxisFinalOutput = xAxisOutput;
            double yAxisFinalOutput = yAxisOutput;
            double turnFinalOutput = 0;
        
        // AUTO CORRECT
            if (getAngleZ() != 0) {
                turnFinalOutput = 2 * ((setAngle - getAngleZ()) / autoCorrectAngle);  
            }
            if (turnFinalOutput > maxSpinSpeed) {turnFinalOutput = maxSpinSpeed;}          
            if (turnFinalOutput < -maxSpinSpeed) {turnFinalOutput = -maxSpinSpeed;} 

            frontLeft.set(yAxisFinalOutput - xAxisFinalOutput + turnFinalOutput);
            frontRight.set(yAxisFinalOutput + xAxisFinalOutput - turnFinalOutput);
            backRight.set(yAxisFinalOutput - xAxisFinalOutput - turnFinalOutput);
            backLeft.set(yAxisFinalOutput + xAxisFinalOutput + turnFinalOutput);
       
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
