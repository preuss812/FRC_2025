// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;

/** 
 * This class supplies data to create trajectories from any point on the field to the Processor or SOURCE
 * The hope is that these will enable semi-automatic driving from to and from those key field positions.
 * The strategy is the divide the field into 2 meter squares and then create plans based on the
 * best path to travel from each starting square.
 */
public class TrajectoryPlans {

    private static boolean debug = false;

    public enum FieldStep {
        Done,
        Up,
        UpLeft,
        UpRight,
        Down,
        DownLeft,
        DownRight,
        Left,
        Right
    }
    public static FieldStep transformFieldStep(FieldStep step)
    {
        FieldStep result = step;
        switch (step) {
            case Done:
                result = FieldStep.Done;
                break;
            case Up:
                result = FieldStep.Up;
                break;
            case UpLeft:
                result = FieldStep.UpRight;
                break;
            case UpRight:
            result = FieldStep.UpLeft;
                break;
            case Down:
            result = FieldStep.Down;
                break;
            case DownLeft:
            result = FieldStep.DownRight;
                break;
            case DownRight:
            result = FieldStep.DownLeft;
                break;
            case Left:
            result = FieldStep.Right;
                break;
            case Right:
            result = FieldStep.Left;
                break;
            default:
                result = step;
        }
        return result;
    }
    public static class FieldSquare {
        public Translation2d waypoint;
        public FieldStep move;
        FieldSquare(Translation2d waypoint, FieldStep move) {
            this.waypoint = waypoint;
            this.move = move;
        }
    }
    public static class TrajectoryPlan {
        public FieldSquare[][] plan;
        public TrajectoryPlan(FieldSquare[][] plan) {
            this.plan = plan;
        }
    }

    public static TrajectoryPlan transformPlan(TrajectoryPlan trajectoryPlan, Transform2d transform) {
        TrajectoryPlan newTrajectoryPlan;
        FieldSquare[][] newPlan = new FieldSquare[8][4];
        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < 4; j++) {
                Translation2d newWaypoint = new Translation2d(FieldConstants.xMax - trajectoryPlan.plan[i][j].waypoint.getX(), trajectoryPlan.plan[i][j].waypoint.getY());
                FieldStep newMove = transformFieldStep(trajectoryPlan.plan[i][j].move);
                //newPlan[7-i][j] = new TrajectoryPlans.FieldSquare( trajectoryPlan.plan[7-i][j].center, newMove);
                newPlan[7-i][j] = new TrajectoryPlans.FieldSquare(newWaypoint, newMove);
            }
        }
        newTrajectoryPlan = new TrajectoryPlan(newPlan);
        return newTrajectoryPlan;
    }
    public static double dx = FieldConstants.xMax/8.0;
    public static double dy = FieldConstants.yMax/4.0;
    public static final TrajectoryPlan BlueProcessorPlan = new TrajectoryPlan( new FieldSquare[][]
        {
            { // Colunm 0:
                new FieldSquare(new Translation2d(dx*1.5, dy*0.5), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*1.5, dy*0.5), FieldStep.DownRight),
                new FieldSquare(new Translation2d(dx*0, dy*2+1.0), FieldStep.Down),
                new FieldSquare(new Translation2d(dx*0, dy*3+0.75), FieldStep.Down)
            },
            { // Column 1:
                new FieldSquare(new Translation2d(dx*1+1.0, dy*0+2.0), FieldStep.UpLeft),
                new FieldSquare(new Translation2d(dx*1+1.0, dy*1+1.5), FieldStep.Left),
                new FieldSquare(new Translation2d(dx*1+0.5, dy*2+1.0), FieldStep.Up),
                new FieldSquare(new Translation2d(dx*1+0.0, dy*3+1.0), FieldStep.Done)
            },
            { // Column 2:
                new FieldSquare(new Translation2d(dx*2+0.0, dy*0+2.0), FieldStep.Left),
                new FieldSquare(new Translation2d(dx*2+1.0, dy*1+1.0), FieldStep.DownLeft),
                new FieldSquare(new Translation2d(dx*2+1.0, dy*2+1.0), FieldStep.UpLeft),
                new FieldSquare(new Translation2d(dx*2+1.0, dy*3+1.0), FieldStep.Left)
            },
            { // Column 3:
                new FieldSquare(new Translation2d(dx*3+1.0, dy*0+1.0), FieldStep.Up),
                new FieldSquare(new Translation2d(dx*3+1.0, dy*1+1.0), FieldStep.Up),
                new FieldSquare(new Translation2d(dx*3+1.0, dy*2+1.5), FieldStep.Up),
                new FieldSquare(new Translation2d(dx*3+0.5, dy*3+0.5), FieldStep.Left)
            },
            { // Column 4:
                new FieldSquare(new Translation2d(dx*4+1.0, dy*0+1.0), FieldStep.UpLeft),
                new FieldSquare(new Translation2d(dx*4+0.0, dy*1+1.0), FieldStep.UpLeft),
                new FieldSquare(new Translation2d(dx*4+0.0, dy*2+1.0), FieldStep.UpLeft),
                new FieldSquare(new Translation2d(dx*4+2.0, dy*3+1.0), FieldStep.Left)
            },
            { // Column 5:
                new FieldSquare(new Translation2d(dx*5+1.0, dy*0+1.0), FieldStep.Left),
                new FieldSquare(new Translation2d(dx*5+1.0, dy*1+1.0), FieldStep.DownRight),
                new FieldSquare(new Translation2d(dx*5+1.0, dy*2+1.0), FieldStep.Up),
                new FieldSquare(new Translation2d(dx*5+1.0, dy*3+1.0), FieldStep.Left),
            },
            { // Column 6:
                new FieldSquare(new Translation2d(dx*6+1.0, dy*0+1.0), FieldStep.Left),
                new FieldSquare(new Translation2d(dx*6+1.0, dy*1+1.0), FieldStep.DownLeft), // To avoid the pillars of the Stage
                new FieldSquare(new Translation2d(dx*6+1.0, dy*2+1.0), FieldStep.UpLeft),
                new FieldSquare(new Translation2d(dx*6+1.0, dy*3+1.0), FieldStep.Left)
            },
            { // Column 7:
                new FieldSquare(new Translation2d(dx*7+0.0, dy*0+1.0), FieldStep.Left),
                new FieldSquare(new Translation2d(dx*7+0.0, dy*1+1.0), FieldStep.DownLeft),
                new FieldSquare(new Translation2d(dx*7+0.0, dy*2+1.0), FieldStep.UpLeft),
                new FieldSquare(new Translation2d(dx*7+0.0, dy*3+1.0), FieldStep.Left)
            }        
    });
    public static final TrajectoryPlan BlueSourcePlan = new TrajectoryPlan( new FieldSquare[][]
        {
            
            /* This version is smoother but sometimes crashes the tragectorygenerator. */
            { // Colunm 0:
                new FieldSquare(new Translation2d(dx*0+2.0, dy*0+2.0), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*0+1.5, dy*1+1.0), FieldStep.DownRight),
                new FieldSquare(new Translation2d(dx*0+2.0, dy*2+1.0), FieldStep.Down),
                new FieldSquare(new Translation2d(dx*0+2.0, dy*3+1.0), FieldStep.Right)
            },
            { // Column 1:
                new FieldSquare(new Translation2d(dx*1+1.0, dy*0+1.0), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*1+1.0, dy*1+0.0), FieldStep.DownRight),
                new FieldSquare(new Translation2d(dx*1+0.5, dy*2+1.0), FieldStep.UpRight),
                new FieldSquare(new Translation2d(dx*1+1.0, dy*3+1.0), FieldStep.Right)
            },
            { // Column 2:
                new FieldSquare(new Translation2d(dx*2+1.0, dy*0+1.0), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*2+1.0, dy*1+1.0), FieldStep.Down),
                new FieldSquare(new Translation2d(dx*2+1.0, dy*2+1.0), FieldStep.Up),
                new FieldSquare(new Translation2d(dx*2+1.0, dy*3+1.0), FieldStep.Right)
            },
            { // Column 3:
                new FieldSquare(new Translation2d(dx*3+1.0, dy*0+1.0), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*3+1.0, dy*1+1.0), FieldStep.DownRight),
                new FieldSquare(new Translation2d(dx*3+1.0, dy*2+1.0), FieldStep.DownRight),
                new FieldSquare(new Translation2d(dx*3+0.5, dy*3+0.0), FieldStep.Down)
            },
            { // Column 4:
                new FieldSquare(new Translation2d(dx*4+1.0, dy*0+1.0), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*4+0.0, dy*1+1.0), FieldStep.Down),
                new FieldSquare(new Translation2d(dx*4+0.5, dy*2+1.0), FieldStep.Down),
                new FieldSquare(new Translation2d(dx*4+1.0, dy*3+1.0), FieldStep.Down)
            },
            { // Column 5:
                new FieldSquare(new Translation2d(dx*5+1.0, dy*0+1.0), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*5+1.0, dy*1+1.0), FieldStep.DownRight),
                new FieldSquare(new Translation2d(dx*5+2.0, dy*2+1.0), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*5+1.0, dy*3+1.0), FieldStep.DownRight),
            },
            { // Column 6:
                new FieldSquare(new Translation2d(dx*6+2.0, dy*0+1.0), FieldStep.Done),
                new FieldSquare(new Translation2d(dx*6+2.0, dy*1+0.0), FieldStep.Down), 
                new FieldSquare(new Translation2d(dx*6+1.0, dy*2+1.0), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*6+1.0, dy*3+1.0), FieldStep.Down)
            },
            { // Column 7:
                new FieldSquare(new Translation2d(dx*7+1.0, dy*0+1.0), FieldStep.Done),
                new FieldSquare(new Translation2d(dx*7+1.0, dy*1+1.0), FieldStep.DownLeft),
                new FieldSquare(new Translation2d(dx*7+0.5, dy*2+0.0), FieldStep.DownLeft),
                new FieldSquare(new Translation2d(dx*7+1.0, dy*3+1.0), FieldStep.Down)
            }   
                 
            /*
            { // Colunm 0:
                new FieldSquare(new Translation2d(dx*0+2.0, dy*0+2.0), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*0+1.5, dy*1+1.0), FieldStep.DownRight),
                new FieldSquare(new Translation2d(dx*0+2.0, dy*2+1.0), FieldStep.Down),
                new FieldSquare(new Translation2d(dx*0+1.0, dy*3+1.0), FieldStep.Right)
            },
            { // Column 1:
                new FieldSquare(new Translation2d(dx*1+1.0, dy*0+1.0), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*1+1.0, dy*1+0.0), FieldStep.DownRight),
                new FieldSquare(new Translation2d(dx*1+0.5, dy*2+1.0), FieldStep.UpRight),
                new FieldSquare(new Translation2d(dx*1+1.0, dy*3+1.0), FieldStep.Right)
            },
            { // Column 2:
                new FieldSquare(new Translation2d(dx*2+1.0, dy*0+1.0), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*2+1.0, dy*1+1.0), FieldStep.DownRight),
                new FieldSquare(new Translation2d(dx*2+1.0, dy*2+1.0), FieldStep.Up),
                new FieldSquare(new Translation2d(dx*2+1.0, dy*3+1.0), FieldStep.Right)
            },
            { // Column 3:
                new FieldSquare(new Translation2d(dx*3+1.0, dy*0+1.0), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*3+1.0, dy*1+1.0), FieldStep.DownRight),
                new FieldSquare(new Translation2d(dx*3+1.0, dy*2+1.0), FieldStep.DownRight),
                new FieldSquare(new Translation2d(dx*3+1.0, dy*3+1.0), FieldStep.Down)
            },
            { // Column 4:
                new FieldSquare(new Translation2d(dx*4+1.0, dy*0+1.0), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*4+1.0, dy*1+1.0), FieldStep.Down),
                new FieldSquare(new Translation2d(dx*4+0.5, dy*2+1.0), FieldStep.Down),
                new FieldSquare(new Translation2d(dx*4+1.0, dy*3+1.0), FieldStep.Down)
            },
            { // Column 5:
                new FieldSquare(new Translation2d(dx*5+1.0, dy*0+1.0), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*5+1.0, dy*1+1.0), FieldStep.DownRight),
                new FieldSquare(new Translation2d(dx*5+2.0, dy*2+1.0), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*5+1.0, dy*3+1.0), FieldStep.DownRight),
            },
            { // Column 6:
                new FieldSquare(new Translation2d(dx*6+1.0, dy*0+1.0), FieldStep.Done),
                new FieldSquare(new Translation2d(dx*6+2.0, dy*1+0.0), FieldStep.Down), 
                new FieldSquare(new Translation2d(dx*6+1.0, dy*2+1.0), FieldStep.DownRight),
                new FieldSquare(new Translation2d(dx*6+1.0, dy*3+1.0), FieldStep.Down)
            },
            { // Column 7:
                new FieldSquare(new Translation2d(dx*7+1.0, dy*0+1.0), FieldStep.Done),
                new FieldSquare(new Translation2d(dx*7+1.0, dy*1+1.0), FieldStep.Down),
                new FieldSquare(new Translation2d(dx*7+0.5, dy*2+0.0), FieldStep.Down),
                new FieldSquare(new Translation2d(dx*7+1.0, dy*3+1.0), FieldStep.Down)
            }  
            */
                  
    });
    public static final TrajectoryPlan RedProcessorPlan = transformPlan(BlueProcessorPlan, FieldConstants.AllianceTransformation[FieldConstants.RedAlliance]);
    public static final TrajectoryPlan RedSourcePlan = transformPlan(BlueSourcePlan, FieldConstants.AllianceTransformation[FieldConstants.RedAlliance]);
    public TrajectoryPlans() {

    }
    public static List<Translation2d> planTrajectory(TrajectoryPlans.TrajectoryPlan trajectoryPlan, Pose2d pose) {
        List<Translation2d> list = new ArrayList<>();
        // convert the coordinates into indexes for 2 by 2 meter squares.
        // 0,0 is the lower left of the field by the Red Alliance source.
        int i = MathUtil.clamp((int)(pose.getX()/TrajectoryPlans.dx),0,7);
        int j = MathUtil.clamp((int)(pose.getY()/TrajectoryPlans.dy),0,3);
        //int n = 0;
        //int maxN = 4;
        String moves = "("+i+","+j+"),";
        TrajectoryPlans.FieldStep move = trajectoryPlan.plan[i][j].move;
        while (move != TrajectoryPlans.FieldStep.Done) {
            if (move == TrajectoryPlans.FieldStep.Left || move == TrajectoryPlans.FieldStep.UpLeft || move == TrajectoryPlans.FieldStep.DownLeft)
                i = i - 1;
            if (move == TrajectoryPlans.FieldStep.Right || move == TrajectoryPlans.FieldStep.UpRight || move == TrajectoryPlans.FieldStep.DownRight)
                i = i + 1;
            if (move == TrajectoryPlans.FieldStep.Down || move == TrajectoryPlans.FieldStep.DownLeft || move == TrajectoryPlans.FieldStep.DownRight)
                j = j - 1;
             if (move == TrajectoryPlans.FieldStep.Up || move == TrajectoryPlans.FieldStep.UpLeft || move == TrajectoryPlans.FieldStep.UpRight)
                j = j + 1;
                if (debug) SmartDashboard.putString("move","i="+i+" j="+j+" move="+move);
            if (i < 0 || i > 7 || j < 0 || j > 7) {
                //int x = 5;
            }
            if (move != TrajectoryPlans.FieldStep.Done) {
                list.add(trajectoryPlan.plan[i][j].waypoint);
                move = trajectoryPlan.plan[i][j].move;
                moves = moves+ "("+i+","+j+"),";
                //if (n++ > maxN) break;
            }
        }
        if (debug) SmartDashboard.putString("TTM",moves);
        return list;
    }
}
