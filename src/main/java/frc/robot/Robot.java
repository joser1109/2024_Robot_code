// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
//The shit above this is stuff imported from the REVLIB vender library
import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private static final int PH_CAN_ID = 11;
  PneumaticHub m_ph = new PneumaticHub(PH_CAN_ID);
  public static int forwardChannel1 = 0;
  public static int reverseChannel1 = 1;
  DoubleSolenoid m_doubleSolenoid = m_ph.makeDoubleSolenoid(forwardChannel1, reverseChannel1);

  CANVenom FrontMotorRight = new CANVenom(1);
  CANVenom RearMotorRight = new CANVenom(2);
  CANVenom RearMotorLeft = new CANVenom(3);
  CANVenom FrontMotorLeft = new CANVenom(4);
  // The motors above are for tank drive
  CANSparkMax MotorMotor = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax MotoMoto = new CANSparkMax(6, MotorType.kBrushless);
  CANSparkMax Janet = new CANSparkMax(7, MotorType.kBrushless);
  CANSparkMax Brock = new CANSparkMax(8, MotorType.kBrushless);

  // Motors for sucking and shooting
  // The shity motors now have a name and a set number
  CANSparkMax InputMotor = new CANSparkMax(9, MotorType.kBrushless);
  Spark LiftyUppy = new Spark(0);
  XboxController Xboob = new XboxController(0);
  // The Xbox controller is now the Xboob🤤🤤🤤

  public void setDriveMotors(double forward, double turn) {
    SmartDashboard.putNumber("drive forward power (%)", forward);
    SmartDashboard.putNumber("drive turn power (%)", turn);

    double left = forward - turn;
    double right = forward + turn;

    SmartDashboard.putNumber("drive turn power (%)", left);
    SmartDashboard.putNumber("drive turn power (%)", right);

    FrontMotorLeft.set(left);
    RearMotorLeft.follow(FrontMotorLeft);
    FrontMotorRight.set(right);
    RearMotorRight.follow(FrontMotorRight);

  }

  public void suckysucky(double suck, double unsuck) {
    SmartDashboard.putNumber("drive suck power (%)", suck);
    SmartDashboard.putNumber("drive unsuck power (%)", unsuck);

    double supersuck = suck + unsuck;

    MotoMoto.set(supersuck);
    MotorMotor.set(supersuck);
    Brock.set(-supersuck);
    Janet.set(-supersuck);
    MotoMoto.setOpenLoopRampRate(0);
    MotorMotor.setOpenLoopRampRate(0);
    Janet.setOpenLoopRampRate(0);
    Brock.setOpenLoopRampRate(0);

  }

  /**
   * Okay so above just tells the motors what to do in teleoperated
   */

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    new RobotContainer();

    SmartDashboard.setDefaultBoolean("Enable Compressor Digital", false);
    SmartDashboard.setDefaultBoolean("Disable Comptessor", false);

    SmartDashboard.setDefaultBoolean("Set Solenoid", false);
    SmartDashboard.setDefaultBoolean("Set Off", false);
    SmartDashboard.setDefaultBoolean("Set Forward", false);
    SmartDashboard.setDefaultBoolean("Set Reverse", false);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putBoolean("Digital Pressure Switch", m_ph.getPressureSwitch());

    SmartDashboard.putBoolean("Compressor Running", m_ph.getCompressor());

    if (SmartDashboard.getBoolean("Enable Compressor", false)) {
      SmartDashboard.putBoolean("Enable Compressor Digital", false);

      m_ph.enableCompressorDigital();

    }

    if (SmartDashboard.getBoolean("Disable Compressor", false)) {
      SmartDashboard.putBoolean("Disable Compressor", false);

      m_ph.disableCompressor();
    }

    CommandScheduler.getInstance().run();

    switch (m_doubleSolenoid.get()) {
      case kOff:
        SmartDashboard.putString("Get Solenoid", "kOff");
        break;
      case kForward:
        SmartDashboard.putString("Get Solenoid", "kForward");
        break;
      case kReverse:
        SmartDashboard.putString("Get Solenoid", "kReverse");
        break;
      default:
        SmartDashboard.putString("Get Solenoid", "N/A");
        break;
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    FrontMotorLeft.setBrakeCoastMode(BrakeCoastMode.Brake);
    RearMotorLeft.follow(FrontMotorLeft);
    FrontMotorRight.setBrakeCoastMode(BrakeCoastMode.Brake);
    RearMotorRight.follow(FrontMotorRight);
    // The motors above are for tank drive
    MotoMoto.set(0);
    MotorMotor.set(0);
    Janet.set(0);
    Brock.set(0);
    // Motors for sucking and shooting
  }

  @Override
  public void disabledPeriodic() {

    FrontMotorLeft.setBrakeCoastMode(BrakeCoastMode.Brake);
    RearMotorLeft.follow(FrontMotorLeft);
    FrontMotorRight.setBrakeCoastMode(BrakeCoastMode.Brake);
    RearMotorRight.follow(FrontMotorRight);
    // The motors above are for tank drive
    MotoMoto.setIdleMode(IdleMode.kBrake);
    MotorMotor.setIdleMode(IdleMode.kBrake);
    Janet.setIdleMode(IdleMode.kBrake);
    Brock.setIdleMode(IdleMode.kBrake);

  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    FrontMotorLeft.setBrakeCoastMode(BrakeCoastMode.Brake);
    RearMotorLeft.setBrakeCoastMode(BrakeCoastMode.Brake);
    FrontMotorRight.setBrakeCoastMode(BrakeCoastMode.Brake);
    RearMotorRight.setBrakeCoastMode(BrakeCoastMode.Brake);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double elapsedTime = Timer.getMatchTime();

    // Run the first set of actions for the first 5 seconds
    if (elapsedTime < 1) {
      MotoMoto.set(0.5);
      MotorMotor.set(0);
      Janet.set(-0.5);
      Brock.set(0);
    }

    if (elapsedTime < 3) {
      MotoMoto.set(0.5);
      MotorMotor.set(0.5);
      Janet.set(-0.5);
      Brock.set(-0.5);
    }

    if (elapsedTime < 7) {
      FrontMotorLeft.set(-0.4);
      RearMotorLeft.follow(FrontMotorLeft);
      FrontMotorRight.set(0.4);
      RearMotorRight.follow(FrontMotorRight);
    }

    // Stop all actions after 15 seconds
    else {
      FrontMotorLeft.set(0);
      RearMotorLeft.follow(FrontMotorLeft);
      FrontMotorRight.set(0);
      RearMotorRight.follow(FrontMotorRight);
      MotoMoto.set(0);
      MotorMotor.set(0);
      Janet.set(0);
      Brock.set(0);
    }
  }

  @Override
  public void teleopInit() {

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
    suckysucky(Xboob.getLeftTriggerAxis(), -Xboob.getRightTriggerAxis());

    if (Xboob.getAButton()) {
      MotoMoto.set(0.35);
      Brock.set(-0.35);
      MotorMotor.set(0);
      Janet.set(0);
      InputMotor.set(-0.35);
    } else if (Xboob.getBButton()) {
      MotoMoto.set(0.35);
      Brock.set(-0.35);
      MotorMotor.set(0);
      Janet.set(0);
      InputMotor.set(0.35);
    } else if (Xboob.getXButton()) {
      MotoMoto.set(0.35);
      MotorMotor.set(0.35);
      Janet.set(-0.35);
      Brock.set(-0.35);
    } else if (Xboob.getYButton()) {
      MotoMoto.set(0.35);
      MotorMotor.set(0.35);
      Janet.set(-0.35);
      Brock.set(-0.35);
    } else {
      InputMotor.set(0);
    }
    if (Xboob.getRawButton(5)) {
      m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    if (Xboob.getRawButton(6)) {
      m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    if (Xboob.getStartButton()) {
      LiftyUppy.set(1);
      LiftyUppy.setInverted(false);
    } else if (Xboob.getBackButton()) {
      LiftyUppy.set(1);
      LiftyUppy.setInverted(true);
    } else {
      LiftyUppy.set(0);
    }

    if (SmartDashboard.getBoolean(
        "Set Forward", false)) {
      SmartDashboard.putBoolean(
          "Set Forward", false);

      m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    if (SmartDashboard.getBoolean(
        "Set Reverse", false)) {
      SmartDashboard.putBoolean("Set Reverse", false);

      m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);

    }

  }

  // The shit above this is simply put the motors getting input from the
  // controller joysticks
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}