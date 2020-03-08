/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.FastAim;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Powercell.Aimer;
import frc.robot.subsystems.Powercell.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoAim extends SequentialCommandGroup {
  /**
   * Creates a new AutoAim.
   */
  public AutoAim(Turret turret,Vision vision,Aimer aimer) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
    new FastAim(turret,vision),
    new InstantCommand(()->aimer.getunit(), aimer),
    new StartEndCommand(()->aimer.aimunit(),()->aimer.isAimfinish(), aimer).withInterrupt(()->aimer.isAimfinish()));
  }
}
