/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.Powercell;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Add your docs here.
 */

public class Sensor {
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-unicorn");
    private static double y, x;


    public static double getlimelightDist() {
 
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry ty= table.getEntry("ty");

        if(ta.getDouble(0.0)>0){
        y = ty.getDouble(0.0);
        }
        
        return (227-50)/Math.tan(Math.toRadians(41+y));

    }
    public static double getlimelightTx(){
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tx= table.getEntry("tx");

        if(ta.getDouble(0.0)>0){
        x = tx.getDouble(0.0);
        }
        return x;
    }
    
}
