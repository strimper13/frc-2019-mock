package frc.team166.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team166.robot.Robot;
import frc.team166.robot.RobotMap;
import frc.team166.robot.RobotMap.PreferenceStrings;
import frc.team166.chopshoplib.commands.CommandChain;
import frc.team166.chopshoplib.commands.SubsystemCommand;
import frc.team166.chopshoplib.sensors.Lidar;
import edu.wpi.first.wpilibj.I2C.Port;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Shooter extends Subsystem {

    WPI_VictorSPX m_leftshoot = new WPI_VictorSPX(RobotMap.CAN.LEFT);
    WPI_VictorSPX m_rightshoot = new WPI_VictorSPX(RobotMap.CAN.RIGHT);

    /**
     * defines the left and right motors defined above into a differential drive
     * that can be used for arcade and tank drive, amung other things
     */
    DifferentialDrive m_shooter = new DifferentialDrive(m_leftshoot, m_rightshoot);

    public Shooter() {

        addChild(m_shooter);

    }

    public void initDefaultCommand() {
        setDefaultCommand(ShooterHandControl());

    }

    public void reset() {
        m_shooter.stopMotor();
    }

    public Command ShooterHandControl() {
        return new SubsystemCommand("ShooterHandControl", this) {
            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void execute() {
                // m_shooter.arcadeDrive(
                // (Robot.m_oi.xBoxDrive.getTriggerAxis(Hand.kRight)
                // - Robot.m_oi.xBoxDrive.getTriggerAxis(Hand.kLeft)),
                // Robot.m_oi.xBoxDrive.getX(Hand.kLeft));

            }

        };
    }

};
