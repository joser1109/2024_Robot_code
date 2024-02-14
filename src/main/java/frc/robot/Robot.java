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
import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
CANVenom Shrek = new CANVenom(1);
CANVenom Vessel = new CANVenom(2);
CANVenom Wyatt = new CANVenom(3);
CANVenom Furry = new CANVenom(4); 
 //The motors above are for tank drive
CANSparkMax MotorMotor = new CANSparkMax(5, MotorType.kBrushless);
CANSparkMax MotoMoto = new CANSparkMax(6, MotorType.kBrushless);
CANSparkMax Mommy = new CANSparkMax(7, MotorType.kBrushless);
CANSparkMax Brock = new CANSparkMax(8, MotorType.kBrushless);
CANSparkMax SnowBlower = new CANSparkMax(9, MotorType.kBrushed);
//Motors for sucking and shooting
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

Shrek.set(right);
Vessel.set(right);
Wyatt.set(right);
Furry.set(right);

}

public void suckysucky (double suck , double unsuck) {
  SmartDashboard.putNumber("drive suck power (%)", suck);
  SmartDashboard.putNumber("drive unsuck power (%)", unsuck);

  double supersuck = suck + unsuck;
 
  MotoMoto.set(supersuck);
  MotorMotor.set(supersuck);
  Brock.set(-supersuck);
  Mommy.set(-supersuck);
MotoMoto.setOpenLoopRampRate(15);
MotorMotor.setOpenLoopRampRate(0);
Mommy.setOpenLoopRampRate(0.8);
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
    Furry.setBrakeCoastMode(BrakeCoastMode.Brake);
    Vessel.setBrakeCoastMode(BrakeCoastMode.Brake);
    Wyatt.setBrakeCoastMode(BrakeCoastMode.Brake);
    Furry.setBrakeCoastMode(BrakeCoastMode.Brake); 
    //The motors above are for tank drive
    MotoMoto.set(0);
    MotorMotor.set(0);  
    Mommy.set(0);
    Brock.set(0);
    //Motors for sucking and shooting
  }

  @Override
  public void disabledPeriodic() { 
    Furry.setBrakeCoastMode(BrakeCoastMode.Coast);
    Vessel.setBrakeCoastMode(BrakeCoastMode.Coast);
    Wyatt.setBrakeCoastMode(BrakeCoastMode.Coast);
    Furry.setBrakeCoastMode(BrakeCoastMode.Coast);
       //The motors above are for tank drive
       MotoMoto.setIdleMode(IdleMode.kBrake);
       MotorMotor.setIdleMode(IdleMode.kBrake);
       Mommy.setIdleMode(IdleMode.kBrake);
       Brock.setIdleMode(IdleMode.kBrake);
       //The motors above are for sucking and shooting
   
}
  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    
    Shrek.set(0.10);
    Vessel.set(0.10);
    Wyatt.set(0.10);
    Furry.set(0.10);
    
   
    Shrek.setInverted(true);
    Vessel.setInverted(true);
    Wyatt.setInverted(false);
    Furry.setInverted(false);
    try { Thread.sleep(2000);
    } catch (InterruptedException b) {
    b.printStackTrace();
    }

   



    Shrek.set(0.10);
    Vessel.set(0.10);
    Wyatt.set(0.10);
    Furry.set(0.10);

    ;

    Shrek.setInverted(false);
    Vessel.setInverted(false);
    Wyatt.setInverted(true);
    Furry.setInverted(true);
 try { Thread.sleep(10000);
} catch (InterruptedException e) {
  e.printStackTrace();
}
    Shrek.set(0);
    Vessel.set(0);
    Wyatt.set(0);
    Furry.set(0);



  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {

  Shrek.setMaxAcceleration(60);
  Vessel.setMaxAcceleration(60);
  Wyatt.setMaxAcceleration(60);
  Furry.setMaxAcceleration(60);

 
    
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
    setDriveMotors(Xboob.getRightX(), Xboob.getLeftY());
    suckysucky(Xboob.getLeftTriggerAxis(),-Xboob.getRightTriggerAxis());

    
    if (Xboob.getAButton()) {
      MotoMoto.set(0.75);
    
      Brock.set(-0.75);
    
      SnowBlower.set(0.75);
    } else if (Xboob.getBButton()) {
      MotoMoto.set(-0.75);
   
      Brock.set(0.75);
    
      SnowBlower.set(-0.75);     
    } else {
      SnowBlower.set(0);
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
