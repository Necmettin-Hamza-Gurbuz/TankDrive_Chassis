// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  private final Joystick  m_controller = new Joystick(0);
  private Timer m_timer = new Timer();
  private final WPI_VictorSPX sol1 = new WPI_VictorSPX(1);
  private final WPI_VictorSPX sol2 = new WPI_VictorSPX(2);
  private final WPI_VictorSPX sag1 = new WPI_VictorSPX(3);
  private final WPI_VictorSPX sag2 = new WPI_VictorSPX(4);
  private final MotorControllerGroup sol = new MotorControllerGroup(sol1,sol2);
  private final MotorControllerGroup sag = new MotorControllerGroup(sag1, sag2);
  private final DifferentialDrive drive = new DifferentialDrive(sol, sag);
  private Encoder encoder;
  private double oncekiPozisyon;
  private double oncekiZaman;
  private double maxspeed = 3300 ;

  private double map(double deger, double eskiMin, double eskiMax, double yeniMin, double yeniMax) {
    return yeniMin + (deger - eskiMin) * (yeniMax - yeniMin) / (eskiMax - eskiMin);
  }

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
   sag.setInverted(true);
   encoder = new Encoder(0, 1);
   encoder.setDistancePerPulse(0.1); // Encoder birimini ayarlayın (örneğin, inç)
    
    m_timer = new Timer();
    m_timer.reset();
    m_timer.start();

    oncekiPozisyon = encoder.getDistance();
    oncekiZaman = m_timer.get();
   
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.restart();
    m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (m_timer.get() < 3.0 && m_timer.get() > 1.0) { // 2 saniye boyunca ileri git
      drive.arcadeDrive(0.6, 0.0); // İleri git, hız 0.5
    } else {
      drive.stopMotor(); // 2 saniye sonra robotu durdur
    }


   
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    drive.arcadeDrive(-m_controller.getRawAxis(1)*0.7, -m_controller.getRawAxis(4)*0.6);
    double guncelPozisyon = encoder.getDistance();
    double guncelZaman = m_timer.get();

    // Hızı hesapla (örneğin, birim/saniye)

    // Hızı hesapla (örneğin, birim/saniye)
    double hiz = (guncelPozisyon - oncekiPozisyon) / (guncelZaman - oncekiZaman);

    double sinirliHiz = Math.min(hiz, maxspeed);
    sinirliHiz = Math.max(sinirliHiz, 0.0); // Hızı 0 ile maksimumHiz arasında sınırla

// Hızı yüzdelik hız oranına dönüştür (0% - 100% arasında)
 double yuzdelikHiz = (sinirliHiz / maxspeed) * 100.0;

System.out.println("Hiz : " + Math.round(yuzdelikHiz) + "%");

  oncekiPozisyon = guncelPozisyon;
  oncekiZaman = guncelZaman; 
    
    
    // Bu değeri bir işlem yaparak kullanabilirsiniz, örneğin ekrana yazdırabilirsiniz.
    

  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic(){
    
  }
}