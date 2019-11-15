package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.GenericHID.Hand;
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

    public void manualDrive() {
        double xAxis = Robot.OI.controller.getX(Hand.kLeft);
        double yAxis = Robot.OI.controller.getY(Hand.kLeft);

        double leftTrigger = Robot.OI.controller.getTriggerAxis(Hand.kLeft);
        double rightTrigger = Robot.OI.controller.getTriggerAxis(Hand.kRight);
        double turn = -leftTrigger + rightTrigger;

        xAxisOutput = xAxis;
        yAxisOutput = yAxis;
        turnOutput = turn / 2;
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
