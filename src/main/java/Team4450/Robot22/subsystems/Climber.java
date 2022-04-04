package Team4450.Robot22.subsystems;

import static Team4450.Robot22.Constants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import Team4450.Lib.SRXMagneticEncoderRelative;
import Team4450.Lib.Util;
import Team4450.Lib.ValveDA;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{

    private WPI_VictorSPX			climberLeftVictor;
    private WPI_TalonSRX            climberRightTalon;

    private MotorControllerGroup climberDrive;

    private DigitalInput climberSwitch = new DigitalInput(CLIMBER_SWITCH);

    private SRXMagneticEncoderRelative  climberEncoder;

	private ValveDA			        mainValve = new ValveDA(MAIN_CLIMBER_VALVE);
    private ValveDA                 auxValve = new ValveDA(AUX_CLIMBER_VALVE);

	private boolean			        brakeEngaged, mainExtended, auxExtended;
	

	

    public Climber()
    {
        Util.consoleLog();

        climberLeftVictor = new WPI_VictorSPX(LEFT_CLIMBER_VICTOR);
		climberRightTalon = new WPI_TalonSRX(RIGHT_CLIMBER_TALON);

        climberLeftVictor.setInverted(true);

        climberLeftVictor.setNeutralMode(NeutralMode.Brake);
	    climberRightTalon.setNeutralMode(NeutralMode.Brake);

        climberDrive = new MotorControllerGroup(climberLeftVictor, climberRightTalon);

        climberEncoder = new SRXMagneticEncoderRelative(climberRightTalon, 1);

        climberEncoder.reset();

        releaseBrake();

        retractMain();
        retractAux();

        Util.consoleLog("Climber created");
    }
    
	public void initialize()
	{
		Util.consoleLog();

		releaseBrake();
        retractMain();
        retractAux();
	}

    @Override
    public void periodic(){

	}

    public void setClimberPower(double power)
    {
        if(power < 0 && climberSwitch.get())
        {
            climberEncoder.reset();
            power = 0;
        }
        
        if(power > 0 && climberEncoder.get() >= 23000) power = 0;

        climberDrive.set(power);
    }
    
    public void stop()
    {
        Util.consoleLog();
        climberDrive.stopMotor();
    }

	public boolean getSwitch()
	{
		return climberSwitch.get();
	}

    public int encoderGet()
    {
        return climberEncoder.get();
    }

    public void engageBrake()
	{
		Util.consoleLog();
		
		brakeEngaged = true;
		
		updateDS();
	}

    public void releaseBrake()
	{
		Util.consoleLog();
		
		//climberBrake.SetB();
		
		brakeEngaged = false;
		
		updateDS();
	}

    public boolean isBrakeEngaged()
	{
		return brakeEngaged;
	}

    public void toggleBrake()
	{
		Util.consoleLog();
		
		if (brakeEngaged)
			releaseBrake();
		else
			engageBrake();
	}
    
    private void updateDS()
	{
		Util.consoleLog();

		SmartDashboard.putBoolean("Brake", brakeEngaged);
		SmartDashboard.putBoolean("Main Extended", mainExtended);
		SmartDashboard.putBoolean("Aux Extended", auxExtended);
	}

    public void extendMain()
	{
		Util.consoleLog();
		
		mainValve.SetA();
        
        mainExtended = true;
	}

    public void retractMain()
	{
		Util.consoleLog();
		
        mainValve.SetB();
        
        mainExtended = false;

        stop();
	}

    public void toggleDeployMain()
	{
		Util.consoleLog("%b", isMainExtended());
		
		if (isMainExtended())
			retractMain();
		else
		  	extendMain();
    }

    
    public boolean isMainExtended()
	{
		return mainExtended;
	}

	public void extendAux()
	{
		Util.consoleLog();
		
		auxValve.SetA();
        
        auxExtended = true;
	}

    public void retractAux()
	{
		Util.consoleLog();
		
        auxValve.SetB();
        
        auxExtended = false;

        stop();
	}

    public void toggleDeployAux()
	{
		Util.consoleLog("%b", isAuxExtended());
		
		if (isAuxExtended())
			retractAux();
		else
		  	extendAux();
    }

    

    public boolean isAuxExtended()
	{
		return auxExtended;
	}
	
}