package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Action {
    String[] action = {};
    double[][] goal = {};

    public void addAction(String newAction, double[] newgoal){
        //adds new action to the action list
        String[] oldaction = action;
        action = new String[oldaction.length+1];
        for (int i = 0; i<oldaction.length; i++){
            action[i] = oldaction[i];
        }
        action[oldaction.length] = newAction;
        
        // adds ther specified distance or amount for the action
        double[][] oldgoal = goal;
        goal = new double[oldgoal.length+1][];
        for (int i = 0; i<oldgoal.length; i++){
            goal[i] = oldgoal[i];
        }
        goal[oldgoal.length] = newgoal;
        
    }

    public void LoadPreset(SendableChooser<String> Preset){
        //resets all current actions
        this.action = new String[] {};
        this.goal = new double[][] {};
        //Uses specified preset
        switch(Preset.getSelected()){
            case "Test":
                this.addAction("drive", new double[] {1}); // displacement
                this.addAction("turn", new double[] {90}); // degrees of turn and nothing
                this.addAction("arc", new double[] {180,1}); // degrees of turn and radius of turn
                this.addAction("turn", new double[] {-270});
                this.addAction("drive", new double[] {1});
                this.addAction("restart", new double[] {});
                break;
            case "Circle":
                this.addAction("drive", new double[] {0.5});
                this.addAction("turn", new double[] {90});
                this.addAction("arc", new double[] {540,0.5});
                this.addAction("turn", new double[] {-270});
                this.addAction("drive", new double[] {0.5});
                break;
            case "Turn":
                this.addAction("turn", new double[] {720});
                break;
            case "Arc":
                this.addAction("arc", new double[] {360, 1});
                break;
            case "Die":
                this.addAction("turn", new double[] {100000});
                break;
        }
        
    }

}
