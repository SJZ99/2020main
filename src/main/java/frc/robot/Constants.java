/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import java.lang.Math;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class DrCon{
        public static final int pidsolt=0;
        public static final int LeftmasterID =18;
        public static final int LeftfollowerID =20;
        public static final int RightmasterID =21;
        public static final int RightfollowerID =19;
        public static final int timeoutMs = 10;
        public static final double rotationPerPulse = 2048;
        public static final double wheeldiameter = 0.1524;
        public static final double kP = 0.03;
        public static final double kF = 0.0506;
        public static final double k = 0.01;
        public static final double Ramptime = 1.5;
        public static final int falconCPR =2048; 
        public static final int maxspeed=10000,maxacc=5000;
        public static final double enoderunit = 2048/(0.1524*Math.PI);

    }
    public static class PowCon{
        //馬達ID
        public static final int aimerID = 8;
        public static final int flywheelID =4;
        public static final int conveyorID = 1;
        public static final int intakearmmasID = 3;
        public static final int turretID = 6;
        public static final int intakeID = 10;
        public static final int wideID = 2;
        public static final int widefolID = 5;
        //限位ID
        public static final int forlimitID =9;
        public static final int backlimitID = 8;


        public static final double rotationPerPulse = 2048;
        public static final int falconCPR =2048;
        public static final int turretCPR =7;
        //flywheel PIDF
        public static final double kP = 0.1;
        public static final double kF = 0.0506;
        
        // 簡單的P控制
        public static final double turretKP = -0.15;
        //aimer的PIDF
        public static final double aimerkF = 0.1;
        public static final double aimerkP = 0.5;
        public static final double aimerkI = 0.01;
        public static final double aimerkD = 0;
        public static final int aimerizone = 400;

        public static final double deadband = 0.001;
        public static final double maxVel = 600;
        public static final double maxAcc = 600;


    /*
        //較複雜的turret Vel PIDF
        public static final double turretVelKF = 6;
        public static final double turretVelKP = 1.2;
        public static final double turretVelkI = 0.1;
        public static final double turrertVelIzone = 50;
*/

        
        public static final int maxspeed=2500,maxacc=1500;




    }
    public static class VisCon{
        public static final double targetheight = 205;
        public static final double limeheight = 50;
        public static final double limeangle = 33;
        public static final double targetDist = 4;
        public static final double threshold = 0.3;
    
    
    }
    public static class CliCon{
        public static final int climbermasID = 0;
        public static final int climberfolID = 4;
        public static final int hookID = 0;
        public static final int[] riseID =new int[]{1,2};

        
    }
    public static class Button{
        // Driverstation
        /**____________________
         * |                  |
         * |    ○ ○  ○  ○     |              
         * |                  |                      
         * |    ○ ○  ○  ○     |
         * |                  |
         * ____________________
         */
        public static final int shoot = 1;

        public static final int aim = 2;
        public static final int turretleft = 5;
        public static final int turretright = 6;

        //Joystick
        public static final int emergencyarmdown  = 7;
        public static final int emergencyarmup    = 8;
        public static final int emergencyintake   = 9;
        public static final int emergencyshooter = 10;
        


    }


}
