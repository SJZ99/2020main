/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.Button;
import frc.robot.commands.Aim;
import frc.robot.commands.DistShooter;
import frc.robot.commands.Auto.Easyauto;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Powercell.Aimer;
import frc.robot.subsystems.Powercell.Arm;
import frc.robot.subsystems.Powercell.Intake;
import frc.robot.subsystems.Powercell.Shooter;
import frc.robot.subsystems.Powercell.Turret;
/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain       m_drivetrain       = new Drivetrain();
  private final Turret           m_turret           = new Turret();
  private final Intake           m_intake           = new Intake();
  private final Shooter          m_shooter          = new Shooter();
  private final Aimer            m_aimer            = new Aimer();
  private final Arm              m_arm              = new Arm();
  private final Vision           m_vision           = new Vision();
  private final Joystick         joystick           = new Joystick(0);
  private final Joystick         drivestation       = new Joystick(1);
  private final Climber          m_climber            = new Climber();
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final Easyauto  m_easyauto  = new  Easyauto(m_turret,m_drivetrain,m_vision,m_intake,m_shooter,m_aimer,m_arm);
  //private final Easyauto1 m_easyauto1 = new Easyauto1(m_drivetrain,m_vision);
  //private final Easyauto2 m_easyauto2 = new Easyauto2(m_shooter,m_drivetrain,m_vision);

  public RobotContainer() {
    
    // Configure the button bindings
    configureButtonBindings();
    m_drivetrain.setDefaultCommand(new RunCommand(()->
    m_drivetrain.curvaturedrive(joystick.getY(), 0.3*joystick.getZ(),true),m_drivetrain));
    m_chooser.addOption("Simple AutoDrive", new InstantCommand(()-> m_drivetrain.drivedist(2),m_drivetrain));


    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    //new JoystickButton(drivestation, Button.turrethoming).whenHeld(new InstantCommand(m_powercell::turrethoming,m_powercell));
    new JoystickButton(drivestation, Button.aim)                  .whenHeld(new Aim(m_turret, m_vision));
    new JoystickButton(drivestation, Button.shoot)                .whenHeld(new DistShooter(m_shooter));
   
    new POVButton(drivestation,0).whenPressed(new InstantCommand(m_arm::armup,m_arm)).whenReleased(new InstantCommand(m_arm::armstop,m_arm));
    new POVButton(drivestation,180).whenPressed(new InstantCommand(m_arm::armdown,m_arm)).whenReleased(new InstantCommand(m_arm::armstop,m_arm));
    new JoystickButton(drivestation, 4).whenPressed(new InstantCommand(m_intake::intake,m_intake)).whenReleased(new InstantCommand(m_intake::intakestop,m_intake));
    new JoystickButton(drivestation, 8).whenHeld(new InstantCommand(()->m_aimer.resetAimer()));
    new JoystickButton(joystick, 11).whenPressed(new InstantCommand(()->m_climber.hookup(), m_climber)).whenReleased(()->m_climber.hookdown());
    new JoystickButton(joystick, 12).whenPressed(new InstantCommand(()->m_climber.rise(), m_climber));
    new JoystickButton(joystick, 10).whenPressed(new InstantCommand(()->m_climber.botclimb(drivestation.getY()), m_climber)).whenReleased(new InstantCommand(()->m_climber.botstopclimb(), m_climber));
    new JoystickButton(joystick, 9).whenPressed(new InstantCommand(()->m_climber.down(), m_climber));
   /* new JoystickButton(joystick,     Button.emergencyintake)      .whenPressed(new InstantCommand(m_intake::intake,m_intake)).whenReleased(new InstantCommand(m_intake::intakestop,m_intake));
    new JoystickButton(joystick,     Button.emergencyarmup)       .whenPressed(new InstantCommand(m_arm::armup,m_arm)).whenReleased(new InstantCommand(m_arm::armstop,m_arm));
    new JoystickButton(joystick,     Button.emergencyarmdown)     .whenPressed(new InstantCommand(m_arm::armdown,m_arm)).whenReleased(new InstantCommand(m_arm::armstop,m_arm));
    */
    new JoystickButton(drivestation, Button.turretleft)           .whenPressed(new InstantCommand(m_turret::turretleft,m_turret)).whenReleased(new InstantCommand(m_turret::turretstop,m_turret));
    new JoystickButton(drivestation, Button.turretright)          .whenPressed(new InstantCommand(m_turret::turretright,m_turret)).whenReleased(new InstantCommand(m_turret::turretstop,m_turret));
    
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_easyauto;
  }
}
