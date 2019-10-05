/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ButtonMap;
import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.RobotMap;
import frc.robot.RobotStats;
import frc.robot.sensors.DriveEncoder;
import frc.robot.tools.controlLoops.PID;
import frc.robot.tools.controlLoops.VelocityPID;
import frc.robot.tools.pathTools.Odometry;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private double deadZone = 0.00;
	private double turn =0;
	private double throttel = 0;
	private double povValue;
	private double ratio = 0;
	private double sensitivity;
	private double minTurnFactor = 0.4;
  public static DriveEncoder leftMainDrive = new DriveEncoder(RobotMap.leftDriveLead,RobotMap.leftDriveLead.getSelectedSensorPosition(0));
  public static DriveEncoder rightMainDrive = new DriveEncoder(RobotMap.rightDriveLead,RobotMap.rightDriveLead.getSelectedSensorPosition(0));
	private double speed;
  private double f = 0.332;
  private double p = 0.171;
  private double i = 0.000000;
  private double d = 17.5;
	private int profile = 0;
	private Odometry autoOdometry;
	private PID alignmentPID;
	private double alignmentP = 0.011;
	private double alignmenti= 0.000;
	private double alignmentd;
	private double power;
	private boolean connected;
	private double distance;
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
	}
	public void startAutoOdometry(double x, double y, double theta){
		autoOdometry = new Odometry(false);
		autoOdometry.setX(x);
		autoOdometry.setY(y);
		autoOdometry.setTheta(theta);
		autoOdometry.start();
	};
	public double getDriveTrainX(){
		return autoOdometry.getX();
	}
	public double getDriveTrainY(){
		return autoOdometry.getY();
	}
	public double getDriveTrainHeading(){
		return autoOdometry.gettheta();
	}
  public void setLowGear(){
    RobotMap.shifters.set(RobotMap.lowGear);
  }
	public void initVelocityPIDs(){
    RobotMap.leftDriveLead.selectProfileSlot(profile, 0);
    RobotMap.leftDriveLead.config_kF(profile, f, 0);
    RobotMap.leftDriveLead.config_kP(profile, p, 0);
    RobotMap.leftDriveLead.config_kI(profile, i, 0);
    RobotMap.leftDriveLead.config_kD(profile, d, 0);
    RobotMap.leftDriveLead.set(ControlMode.Velocity, leftMainDrive.convertftpersToNativeUnitsper100ms(speed));
    RobotMap.rightDriveLead.selectProfileSlot(profile, 0);
    RobotMap.rightDriveLead.config_kF(profile, f, 0);
    RobotMap.rightDriveLead.config_kP(profile, p, 0);
    RobotMap.rightDriveLead.config_kI(profile, i, 0);
    RobotMap.rightDriveLead.config_kD(profile, d, 0);
    RobotMap.rightDriveLead.set(ControlMode.Velocity, rightMainDrive.convertftpersToNativeUnitsper100ms(speed));
	}
	public void initAlignmentPID(){
		alignmentPID = new PID(alignmentP, alignmenti, alignmentd);
		alignmentPID.setMaxOutput(0.4);
		alignmentPID.setMinOutput(-0.4);
    alignmentPID.setSetPoint(0);
	}
  public void setHighGear(){
    RobotMap.shifters.set(RobotMap.highGear);
  }
	public void arcadeDrive(){
		double leftPower;
		double rightPower;
		double differential;
		
		throttel = ButtonMap.getDriveThrottle(); 
		if(throttel ==0){
			throttel = 0.001;
		}
		ratio = Math.abs(1/throttel);
		povValue = ButtonMap.getPOV();
		turn = ButtonMap.getRotation();
		differential = (turn*ratio*sensitivity) + Math.abs(minTurnFactor*turn);

		leftPower = (throttel - (differential));
		rightPower = (throttel + (differential));
	
		if(Math.abs(leftPower)>1) {
			rightPower = Math.abs(rightPower/leftPower)*Math.signum(rightPower);
			leftPower = Math.signum(leftPower);
		}
		else if(Math.abs(rightPower)>1) {
			leftPower = Math.abs(leftPower/rightPower)*Math.signum(leftPower);
			rightPower = Math.signum(rightPower);
		}
    RobotMap.leftDriveLead.set(ControlMode.PercentOutput, leftPower);
    RobotMap.rightDriveLead.set(ControlMode.PercentOutput, rightPower);
		if(ButtonMap.shiftDown()){
			setLowGear();
		}
		else if(ButtonMap.shiftUp()) {
			setHighGear();
		}
		if(RobotMap.shifters.get() == RobotMap.highGear) {
				sensitivity =1;
		}
		else if(RobotMap.shifters.get() == RobotMap.lowGear) {
				sensitivity =1;
    }
	}
	public boolean trackVisionTape(){
    RobotMap.drive.setLowGear();
		Robot.visionCamera.updateVision();
		if(Timer.getFPGATimestamp()-Robot.visionCamera.lastParseTime>0.25){
			alignmentPID.updatePID(0);
		}
		else{
			alignmentPID.updatePID(Robot.visionCamera.getAngle());
		}
		
    power = 0.35;
    RobotMap.drive.setLowGear();
    RobotConfig.setDriveMotorsBrake();
    connected = RobotMap.mainUltrasonicSensor2.isConnected();
    distance = RobotMap.mainUltrasonicSensor2.getDistance();
    if(distance>=1.5&&connected&&distance<7){
      power = Math.pow(distance/15,0.8);
    }
    else if(distance<1.5&&connected){
			power = 0.0;
    }
		RobotMap.leftDriveLead.set(ControlMode.PercentOutput, -power+ alignmentPID.getResult());
		RobotMap.rightDriveLead.set(ControlMode.PercentOutput, -power- alignmentPID.getResult());
		if(ButtonMap.autoBreakTapeTracking()&&RobotState.isAutonomous()){
			return true;
		}
		else{
			return false;
		}
	}
	public void Stop(){
		RobotMap.leftDriveLead.set(ControlMode.PercentOutput, 0);
		RobotMap.rightDriveLead.set(ControlMode.PercentOutput, 0);

	}
		
	public void setLeftSpeed(double speed){
		SmartDashboard.putNumber("output", RobotMap.leftDriveLead.getMotorOutputPercent());
		SmartDashboard.putNumber("target", RobotMap.leftDriveLead.getClosedLoopTarget());
		RobotMap.leftDriveLead.set(ControlMode.Velocity, leftMainDrive.convertftpersToNativeUnitsper100ms(speed));
	}
	public void setRightSpeed(double speed){
		RobotMap.rightDriveLead.set(ControlMode.Velocity, rightMainDrive.convertftpersToNativeUnitsper100ms(speed));

	}
	public void setLeftPercent(double percent){
		RobotMap.leftDriveLead.set(ControlMode.PercentOutput, percent);
	}
	public void setRightPercent(double percent){
		RobotMap.rightDriveLead.set(ControlMode.PercentOutput, percent);
	}
  public void stopDriveTrainMotors(){
    for(TalonSRX talon : RobotMap.driveMotorLeads){
        talon.set(ControlMode.PercentOutput, 0);
    }
  }
}
