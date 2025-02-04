// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import frc.robot.Constants.ColorConstants;

/**
 * This subsystem manages a color detector.
 * Currently, it is set up for the 2024, Crescendo season
 * which had orange foam rings which were detected crossing
 * a black background field.
 * This would need different colors and proximity numbers for reuse.
 * "note" has be replaced by "algae" for 2025 but the colors and distance have not been adjusted.
 */
public class ColorDetectionSubsytem {
  /** Creates a new ColorMatcher. */
    final I2C.Port            i2cPort = I2C.Port.kOnboard;
    final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    final ColorMatch   m_colorMatcher = new ColorMatch();
	private final boolean debug = false;

    public final Color kAlgaeTarget = new Color(
    	ColorConstants.kAlgaeTargetRBG[0], 
		ColorConstants.kAlgaeTargetRBG[1],
		ColorConstants.kAlgaeTargetRBG[2]
    );

	public final Color kBlack = new Color(
		0.0, 0.0, 0.0
	);

    public ColorDetectionSubsytem() {
		m_colorMatcher.addColorMatch(kAlgaeTarget);
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
			if (match.color == kAlgaeTarget) {
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
		return (color == kAlgaeTarget);
    }
}
