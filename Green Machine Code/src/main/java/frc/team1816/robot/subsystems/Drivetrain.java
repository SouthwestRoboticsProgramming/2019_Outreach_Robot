package frc.team1816.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.edinarobotics.utils.checker.CheckFailException;
import com.edinarobotics.utils.checker.Checkable;
import com.edinarobotics.utils.checker.RunTest;
import com.edinarobotics.utils.hardware.RobotFactory;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team1816.robot.Robot;


@RunTest
public class Drivetrain extends Subsystem implements Checkable {
    public static final String NAME = "drivetrain";

    private static final double SLOW_MOD_THROTTLE = 0.5;
    private static final double SLOW_MOD_ROT = 0.8;

    private static double TICKS_PER_REV;
    private static double TICKS_PER_INCH;
    private static double WHEELBASE;
    private static double MAX_VEL_TICKS_PER_100MS;

    private static double INCHES_PER_REV;

    private AHRS navX;

    private IMotorController rightMain;
    private IMotorController rightSlaveOne;
    private IMotorController rightSlaveTwo;

    private IMotorController leftMain;
    private IMotorController leftSlaveOne;
    private IMotorController leftSlaveTwo;

    private double leftPower;
    private double rightPower;
    private double rotation;

    private double leftSetVel;
    private double rightSetVel;

    private double leftMeasPos;
    private double rightMeasPos;
    private double gyroAngle;

    private boolean isPercentOut;
    private boolean isSlowMode;
    private boolean isReverseMode;
    private boolean isVisionMode = false;
    private boolean outputsChanged = false;

    public Drivetrain() {
        super(NAME);
        RobotFactory factory = Robot.factory;

        TICKS_PER_REV = factory.getConstant("ticksPerRev");
        TICKS_PER_INCH = factory.getConstant("ticksPerIn");
        WHEELBASE = factory.getConstant("wheelbase");
        MAX_VEL_TICKS_PER_100MS = factory.getConstant("maxVel");
        INCHES_PER_REV = TICKS_PER_REV / TICKS_PER_INCH;

        this.leftMain = factory.getMotor(NAME, "leftMain");
        this.leftSlaveOne = factory.getMotor(NAME, "leftSlaveOne", leftMain);
        this.leftSlaveTwo = factory.getMotor(NAME, "leftSlaveTwo", leftMain);

        this.rightMain = factory.getMotor(NAME, "rightMain");
        this.rightSlaveOne = factory.getMotor(NAME, "rightSlaveOne", rightMain);
        this.rightSlaveTwo = factory.getMotor(NAME, "rightSlaveTwo", rightMain);

        invertTalons(true);
        setNeutralMode(NeutralMode.Brake);

        try {
            navX = new AHRS(SPI.Port.kMXP);
            System.out.println("NavX Instantiated");
        } catch (RuntimeException e) {
            DriverStation.reportError("Error instantiating navX-MXP:  " + e.getMessage(), true);
        }
    }

    private void invertTalons(boolean invertRight) {
        if (invertRight) {
            this.rightMain.setInverted(true);
            this.rightSlaveOne.setInverted(true);
            this.rightSlaveTwo.setInverted(true);
        } else {
            this.leftMain.setInverted(true);
            this.leftSlaveOne.setInverted(true);
            this.leftSlaveTwo.setInverted(true);
        }
    }

    public void setNeutralMode(NeutralMode mode) {
        this.leftMain.setNeutralMode(mode);
        this.leftSlaveOne.setNeutralMode(mode);
        this.leftSlaveTwo.setNeutralMode(mode);

        this.rightMain.setNeutralMode(mode);
        this.rightSlaveOne.setNeutralMode(mode);
        this.rightSlaveTwo.setNeutralMode(mode);
    }

    public double getLeftPower() {
        return leftPower;
    }

    public double getRightPower() {
        return rightPower;
    }

    public double getGyroAngle() {
        return gyroAngle;
    }

    public boolean getGyroStatus() {
        return navX.isConnected();
    }

    public double getLeftPosTicks() {
        return leftMeasPos;
    }

    public double getRightPosTicks() {
        return rightMeasPos;
    }

    public double ticksToInches(double ticks) {
        return ticks * (1 / TICKS_PER_REV) * INCHES_PER_REV;
    }

    public double getLeftPosInches() {
        return ticksToInches(this.leftMeasPos);
    }

    public double getRightPosInches() {
        return ticksToInches(this.rightMeasPos);
    }

    public void setDrivetrainVelocity(double leftPower, double rightPower) {
        setDrivetrainVelocity(leftPower, rightPower, 0);
    }

    public void setDrivetrainVelocity(double leftPower, double rightPower, double rotation) {
        this.leftPower = leftPower;
        this.rightPower = rightPower;
        this.rotation = rotation;
        isPercentOut = false;
        outputsChanged = true;
    }

    public void setDrivetrainPercent(double leftPower, double rightPower) {
        setDrivetrainPercent(leftPower, rightPower, 0);
    }

    public void setDrivetrainPercent(double leftPower, double rightPower, double rotation) {
        this.leftPower = leftPower;
        this.rightPower = rightPower;
        this.rotation = rotation;
        isPercentOut = true;
        outputsChanged = true;
    }

    public void setDrivetrainVisionNav(boolean visionOn) {
        isVisionMode = visionOn;
    }

    public boolean getVisionStatus() {
        return isVisionMode;
    }

    public void setSlowMode(boolean slowMode) {
        this.isSlowMode = slowMode;
        outputsChanged = true;
    }

    public boolean isReverseMode() {
        return isReverseMode;
    }

    public void setReverseMode(boolean reverseMode) {
        this.isReverseMode = reverseMode;
        outputsChanged = true;
    }

    public void toggleReverseMode() {
        this.isReverseMode = !this.isReverseMode;
        outputsChanged = true;
    }

    @Override
    public void periodic() {
        this.gyroAngle = navX.getAngle();
        this.leftMeasPos = leftMain.getSelectedSensorPosition(0);
        this.rightMeasPos = rightMain.getSelectedSensorPosition(0);

        if (outputsChanged) {
            if (isSlowMode) {
                leftPower *= SLOW_MOD_THROTTLE;
                rightPower *= SLOW_MOD_THROTTLE;
                rotation *= SLOW_MOD_ROT;
            }

            if (isReverseMode) {
                leftPower *= -1;
                rightPower *= -1;
            }

            leftPower += rotation * .55;
            rightPower -= rotation * .55;

            leftSetVel = leftPower * MAX_VEL_TICKS_PER_100MS;
            rightSetVel = rightPower * MAX_VEL_TICKS_PER_100MS;

            if (isPercentOut) {
                this.leftMain.set(ControlMode.PercentOutput, leftPower);
                this.rightMain.set(ControlMode.PercentOutput, rightPower);
            } else {
                this.leftMain.set(ControlMode.Velocity, leftSetVel);
                this.rightMain.set(ControlMode.Velocity, rightSetVel);
            }

            outputsChanged = false;
        }
    }

    @Override
    protected void initDefaultCommand() {
    }

    @Override
    public boolean check() throws CheckFailException {
        // no-op
        return true;
    }
}
