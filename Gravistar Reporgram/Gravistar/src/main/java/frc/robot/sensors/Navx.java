package frc.robot.sensors;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.RobotMap;
import jaci.pathfinder.Pathfinder;


public class Navx {
	private double originalAngle;
	private double originalYaw;
	private double originalRoll;
	private double originalPitch;
	private AHRS imu;

	public Navx(AHRS navx) {
		imu = navx;
		originalAngle = imu.getAngle();
		originalYaw = imu.getYaw();
		originalPitch = imu.getPitch();
		originalRoll = imu.getRoll();
	}
	public Navx(AHRS navx, Double startAngle) {
		imu = navx;
		originalAngle = startAngle;
		originalYaw = imu.getYaw();
		originalPitch = imu.getPitch();
		originalRoll = imu.getRoll();
	}
	public double currentAngle() {
		return imu.getAngle()-originalAngle;	
	}
	public double currentPitch(){
		return imu.getPitch();
	}
	public double currentRoll(){
		return imu.getRoll();
	}
	public double currentYaw(){
		return ((imu.getYaw())-originalYaw);
	}
	public double currentReverseYaw(){
		return Pathfinder.boundHalfDegrees((imu.getYaw())-originalYaw+180) ;
	}
	public boolean isMoving() {
		return imu.isMoving();
	}
	public double currentAccelerometerX(){
		return imu.getWorldLinearAccelX();
	}
	public double currentAccelerometerY(){
		return imu.getWorldLinearAccelY();
	}
	public double currentAccelerometerZ(){
		return imu.getWorldLinearAccelZ();
	}
	public boolean isOn(){
		return imu.isConnected();
	}
	public boolean isMagCalibrated(){
		return imu.isMagnetometerCalibrated();
	}
	public boolean isAutoCalibrating(){
		return imu.isCalibrating();
	}
	public boolean isMagInerference(){
		return imu.isMagneticDisturbance();
	}
	public void softResetAngle(){
		originalAngle = RobotMap.navx.getAngle();

	}
	public void softResetYaw(){
		originalYaw = RobotMap.navx.getYaw();
	}
}
