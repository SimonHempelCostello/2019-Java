/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.Counter;

/**
 * Add your docs here.
 */
public class PWMUltraSonicSensor {
    private Counter sensor;
    public PWMUltraSonicSensor(Counter counter) {
        sensor = counter;
        sensor.setSemiPeriodMode(true);
    }
    public double getDistance(){
        double distance = sensor.getPeriod()*3200.19047619;
        if(distance>1){
            return distance;
        }
        else{
            return -1;
        }
    }
    public boolean isConnected(){
        if(sensor.getPeriod()>1){
            return false;
        }
        else{
            return true;
        }
    }
}
