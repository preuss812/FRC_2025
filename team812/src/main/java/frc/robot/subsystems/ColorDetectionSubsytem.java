// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import frc.robot.Constants.ColorConstants;

public class ColorDetectionSubsytem {
  /** Creates a new ColorMatcher. */
    final I2C.Port            i2cPort = I2C.Port.kOnboard;
    final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    final ColorMatch   m_colorMatcher = new ColorMatch();
	private final boolean debug = false;

    public final Color kNoteTarget = new Color(
    	ColorConstants.kNoteTargetRBG[0], 
		ColorConstants.kNoteTargetRBG[1],
		ColorConstants.kNoteTargetRBG[2]
    );

	public final Color kBlack = new Color(
		0.0, 0.0, 0.0
	);

    public ColorDetectionSubsytem() {
		m_colorMatcher.addColorMatch(kNoteTarget);
		m_colorMatcher.addColorMatch(kBlack);
		m_colorMatcher.setConfidenceThreshold(ColorConstants.kColorConfidenceThreshold);
    }

	public boolean inRange() {
		final int proximity = m_colorSensor.getProximity();

		SmartDashboard.putNumber("Proximity", proximity);
		return (proximity >= 100);
	}
    public Color get_color() {
		final Color detectedColor = m_colorSensor.getColor();
		final int proximity = m_colorSensor.getProximity();
		final ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

		String colorString;

		if (proximity >= 100) {
			if (match.color == kNoteTarget) {
				colorString = "Orange";
			} else if (match.color == kBlack) {
				colorString = "Black";
			} else {
				colorString = "Unknown";
			}
		} else {
			colorString = "Out of range";
		}

		if (debug) {
			SmartDashboard.putNumber("Red", detectedColor.red);
			SmartDashboard.putNumber("Green", detectedColor.green);
			SmartDashboard.putNumber("Blue", detectedColor.blue);
			SmartDashboard.putNumber("Proximity", proximity);
			SmartDashboard.putNumber("Confidence", match.confidence);  
			SmartDashboard.putString("Detected Color", colorString);
		}			
		return match.color;
    }


    public boolean isOrange(Color color) {
		return (color == kNoteTarget);
    }
}
