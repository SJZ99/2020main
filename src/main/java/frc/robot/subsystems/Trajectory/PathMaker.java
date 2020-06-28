/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.Trajectory;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PathMaker extends SubsystemBase {
  /**
   * Creates a new PathMaker.
   */
  static Trajectory trajectory = null;
  public PathMaker() {

  }

  public static Trajectory getTrajectory(String path) {
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException g) {
      System.out.print(g.getMessage());
    }
    return trajectory;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
