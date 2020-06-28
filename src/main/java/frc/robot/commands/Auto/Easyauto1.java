
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto;

import java.io.IOException;
import java.util.Arrays;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Trajectory.PathMaker;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Easyauto1 extends SequentialCommandGroup {
  /**
   * Creates a new Easyauto.
   * 
   * @throws IOException
   */
  public Easyauto1(Drivetrain drivetrain, Vision vision) {
    super(
      new InstantCommand(()->SmartDashboard.putString("Auto Mode", "trajectory")),
      Robot.trajectoryFactory.getRamseteCommand(new Trajectory(Arrays.asList(new Trajectory.State()))));
        
//pathMaker.getTrajectory()
  }
}
