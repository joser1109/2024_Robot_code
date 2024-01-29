// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//The shit above this is already in the FRC WPILIBJ when you download it

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
//The shit above this is stuff imported from the REVLIB vender library
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;


CANSparkMax LeftTopSpark = new CANSparkMax(1, MotorType.kBrushless);
CANSparkMax LeftBottomSpark = new CANSparkMax(2, MotorType.kBrushless);
CANSparkMax RightTopSpark = new CANSparkMax(3, MotorType.kBrushless);
CANSparkMax RightBottomSpark = new CANSparkMax(4, MotorType.kBrushless);
 //The motors above are for tank drive
CANSparkMax MotorMotor = new CANSparkMax(5, MotorType.kBrushless);
CANSparkMax MotoMoto = new CANSparkMax(6, MotorType.kBrushless);
CANSparkMax Janet = new CANSparkMax(7, MotorType.kBrushless);
CANSparkMax Brock = new CANSparkMax(8, MotorType.kBrushless);
 //Motors for sucking and shooting
CANSparkMax Frosty = new CANSparkMax(9, MotorType.kBrushless);
CANSparkMax SnowyGrabby = new CANSparkMax(10, MotorType.kBrushless);
//The shity motors now have a name and a set number

XboxController Xboob = new XboxController(0);
//The Xbox controller is now the Xboob🤤🤤🤤

public void setDriveMotors(double forward, double turn) {
SmartDashboard.putNumber("drive forward power (%)", forward);
SmartDashboard.putNumber("drive turn power (%)", turn);

double left = forward - turn;
double right = forward + turn;

SmartDashboard.putNumber("drive turn power (%)", left);
SmartDashboard.putNumber("drive turn power (%)", right);

LeftTopSpark.set(left);
LeftBottomSpark.set(left);
RightTopSpark.set(right);
RightBottomSpark.set(right);

LeftTopSpark.setOpenLoopRampRate(0.8);
LeftBottomSpark.setOpenLoopRampRate(0.8);
RightTopSpark.setOpenLoopRampRate(0.8);
RightBottomSpark.setOpenLoopRampRate(0.8);

}

public void suckysucky (double suck , double unsuck) {
  SmartDashboard.putNumber("drive suck power (%)", suck);
  SmartDashboard.putNumber("drive unsuck power (%)", unsuck);

  double supersuck = suck + unsuck;
 
  MotoMoto.set(supersuck);
  MotorMotor.set(supersuck);
  Brock.set(-supersuck);
  Janet.set(-supersuck);
MotoMoto.setOpenLoopRampRate(15);
MotorMotor.setOpenLoopRampRate(0);
Janet.setOpenLoopRampRate(0.8);
Brock.setOpenLoopRampRate(15);

}
/**Okay so  above just tells the motors what to do in teleoperated
*/

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    LeftTopSpark.setInverted(false);
    LeftBottomSpark.setInverted(false);
      

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    LeftTopSpark.set(0);
    LeftBottomSpark.set(0);
    RightTopSpark.set(0);
    RightBottomSpark.set(0);  
    //The motors above are for tank drive
    MotoMoto.set(0);
    MotorMotor.set(0);  
    Janet.set(0);
    Brock.set(0);
    //Motors for sucking and shooting
  }

  @Override
  public void disabledPeriodic() {    
    LeftTopSpark.setIdleMode(IdleMode.kCoast);
    LeftBottomSpark.setIdleMode(IdleMode.kCoast);
    RightTopSpark.setIdleMode(IdleMode.kCoast);
    RightBottomSpark.setIdleMode(IdleMode.kCoast);}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {

    LeftTopSpark.setIdleMode(IdleMode.kBrake);
    LeftBottomSpark.setIdleMode(IdleMode.kBrake);
    RightTopSpark.setIdleMode(IdleMode.kBrake);
    RightBottomSpark.setIdleMode(IdleMode.kBrake
    );
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    setDriveMotors(Xboob.getRightX(),-Xboob.getLeftY());
    suckysucky(Xboob.getLeftTriggerAxis(),-Xboob.getRightTriggerAxis());

    if (Xboob.getAButton()) {
      Frosty.set(0.75);
    } else if (Xboob.getBButton()) {
      Frosty.set(-0.75);
    } else {
      Frosty.set(0);
    }

  }
//The shit above this is simply put the motors getting input from the controller joysticks
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
