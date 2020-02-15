`/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
 
  // Sets Defaults
  private static final String kAuto1 = "Auto 1";
  private static final String kAuto2 = "Auto 2";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // Grabs motors and gives them names and ports
  WPI_TalonSRX m_Rmaster = new WPI_TalonSRX(1);
  WPI_TalonSRX m_Lmaster = new WPI_TalonSRX(0);
  VictorSPX m_Rslave = new VictorSPX(1);
  VictorSPX m_Lslave = new VictorSPX(0);

  Talon m_colorWheel = new Talon(5);

  // Changes port To match the colorsensoroutput
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  
  // Makes/maps color sensor
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch colorMatcher = new ColorMatch();

  // Makes color targets
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  // Sets up the drive
  private final DifferentialDrive drive = new DifferentialDrive(m_Rmaster , m_Lmaster);

  // Gives controllers names
  Joystick flightStick = new Joystick(0);
  XboxController gamepad = new XboxController(0);

  // Keep track of colors
  private int colorCount;
  private Color colorSave;
  private boolean colorChange;
  private Color assignedColor;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    // Setup options for autonomous modes
    m_chooser.setDefaultOption("Auto 1", kAuto1);
    m_chooser.addOption("Auto 2", kAuto2);
    SmartDashboard.putData("Auto Choices : ", m_chooser);

    // Reset motors to default
    m_Lmaster.configFactoryDefault();
    m_Rmaster.configFactoryDefault();
    m_Lslave.configFactoryDefault();
    m_Rslave.configFactoryDefault();
    
    // Set our slave motors
    m_Rslave.follow(m_Rmaster);
    m_Lslave.follow(m_Lmaster);

    // Motor behavior
    m_Lmaster.setNeutralMode(NeutralMode.Brake);
    m_Rmaster.setNeutralMode(NeutralMode.Brake);    

    m_Lmaster.set(ControlMode.PercentOutput, 0);
    m_Rmaster.set(ControlMode.PercentOutput, 0);

    drive.setRightSideInverted(true);    
  
    //color
    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget);    

    colorChange = false;

    SmartDashboard.putString("Assigned Color : ", "");

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {    

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    
    Color detectedColor = colorSensor.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    switch (m_autoSelected) {
    case kAuto1:

      SmartDashboard.putString("Auto Status: ", "Running Auto 1");
  
      if (match.color == kBlueTarget){
        drive.stopMotor();
      } else {
        drive.arcadeDrive(.5, 0);
      }

      break;

    case kAuto2:
      // Put custom 2 auto code here
      SmartDashboard.putString("Auto Status: ", "Running Auto 2");
      break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic(){

    Scheduler.getInstance().run();

    getAssignedColorFromGameDate();
    
    flightStickDrive();

    SmartDashboard.putBoolean("Finding Color : ", gamepad.getBButton());
    SmartDashboard.putBoolean("Counting Colors : ", gamepad.getYButton());

    if (gamepad.getBButton()) {
      stopOnColor();
    } else if (gamepad.getYButton()){
      countControlPanelSpins();
    } else {
      m_colorWheel.set(0);
      resetControlPanel();
    }

  }

    /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    Scheduler.getInstance().run();
    
    testColorSensor();

    if (gamepad.getAButton()){
      m_colorWheel.setSpeed(1);
    }else{
      m_colorWheel.setSpeed(0);
    }
  }

  // Sub System Code

  public void getAssignedColorFromGameDate() {
    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B' : 
          assignedColor = kBlueTarget;
          break; 
        case 'G' :
          assignedColor = kGreenTarget;
          break;
        case 'R' :
          assignedColor = kRedTarget;
          break;
        case 'Y' :
          assignedColor = kYellowTarget;
          break;
        default :
          assignedColor = null;
          break;
      }
    } else {
      assignedColor = null;
    }

    if (assignedColor != null) {
      SmartDashboard.putString("Assigned Color : ", getColorString(assignedColor));
    } else {
      SmartDashboard.putString("Assigned Color : ", "Error!");
    }
  }

  // Count a color as it passed by to determine control panel rotations
  public void countControlPanelSpins() {
    ColorMatchResult match = getColorFromSensor();

    if (colorCount < 1) {
      colorSave = match.color;
      colorCount++;
      SmartDashboard.putString("Saved Color : ", getColorString(colorSave));
    }

    if (colorCount >= 8) {
      m_colorWheel.set(0);
    } else {
      m_colorWheel.set(.5);
    }

    if (match.color != colorSave) {
      colorChange = true;
    }

    if (colorChange) {
      // Start looking for original color
      if (match.color == colorSave) {
        // So nice to see you again
        colorChange = false;
        colorCount++;
      }
    }
    SmartDashboard.putNumber("Saved Count : ", colorCount);
  }

  public void resetControlPanel() {
    colorSave = null;
    colorCount = 0;
    colorChange = false;

    SmartDashboard.putString("Saved Color : ", "");
    SmartDashboard.putNumber("Saved Count : ", colorCount);
  }
 
  private void flightStickDrive(){

    double forward = flightStick.getY();
    double turn = flightStick.getTwist();
    double speedFactor = getSpeedFactor();

    forward = Deadband(forward);
    turn = Deadband(turn);

    drive.arcadeDrive(forward * speedFactor * -1, turn * speedFactor);

    SmartDashboard.putString("Drive :: ", "Forward: " + String.format("%.2f", forward) + " || Turn: " + String.format("%.2f", turn) + " || Factor: " + speedFactor);
      
  }

  // Spin motor as long as B button pressed, stop on specified color
  private void stopOnColor() {
    
    // REACH Start a timer for 5 seconds to display on Smartboard    

    ColorMatchResult match = getColorFromSensor();
    Color offsetColor = getOffsetColor(assignedColor);
    if (match.color == offsetColor) {
      m_colorWheel.set(0);
      // TODO Start timer
    } else {
      m_colorWheel.set(.5);
    }
    
  }

  // Order is:  Red, Green, Blue, Yellow
  private Color getOffsetColor(Color assignedColor) {
    
    Color offsetColor = null;

    if (assignedColor == kBlueTarget) {
      offsetColor = kRedTarget;
    } else if (assignedColor == kYellowTarget) {
      offsetColor = kGreenTarget;
    } else if (assignedColor == kRedTarget) {
      offsetColor = kBlueTarget;
    } else {
      offsetColor = kYellowTarget;
    }

    return offsetColor;
  }

  private void testColorSensor() {

    Color detectedColor = colorSensor.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    String colorString = getColorString(match.color);
 
    SmartDashboard.putNumber("Red        : ", detectedColor.red);
    SmartDashboard.putNumber("Green      : ", detectedColor.green);
    SmartDashboard.putNumber("Blue       : ", detectedColor.blue);
    SmartDashboard.putNumber("Confidence : ", match.confidence);
    SmartDashboard.putString("Detected Color >> ", colorString);

  }

  public ColorMatchResult getColorFromSensor() {
    Color detectedColor = colorSensor.getColor();
    return colorMatcher.matchClosestColor(detectedColor);
  }

  public String getColorString(Color color) {
    String colorString;
    if (color == kBlueTarget) {
      colorString = "Blue";
    } else if (color == kRedTarget) {
      colorString = "Red";
    } else if (color == kGreenTarget) {
      colorString = "Green";
    } else if (color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }    
    return colorString;
  }

  private double getSpeedFactor() {
    double factor = flightStick.getRawAxis(3);
    if (factor == -1) {
      factor = 1;
    } else if (factor > -1 && factor < 1) {
      factor = .75;
    } else {
      factor = .5;
    }
    return factor;
  }

  // Stop from using flightstick values that are too small
  private double Deadband(final double value) {
    if (value >= +0.5) {
      return value;
    } else if (value <= -0.5) {
      return value;
    }
    return 0;
  }    

}
