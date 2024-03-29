// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.team5557.state.goal;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.team5557.Constants;
import org.team5557.planners.superstructure.util.SuperstructureState;


public class ObjectiveTracker extends SubsystemBase {
    public GamePiece gamePiece = GamePiece.CUBE; // The selected game piece for intaking and scoring
    public NodeLevel selectedLevel = NodeLevel.HYBRID; // The level of the selected target node
    public Substation selectedSubstation = Substation.DOUBLE_LEFT;

    public int selectedRow = 1; // The row of the selected target node
    public int selectedColumn = 4;

    public ObjectiveTracker() {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.shuffleboard.driver_readout_key);
        
        ShuffleboardLayout layout = tab
            .getLayout("Objective", BuiltInLayouts.kGrid)     
            .withSize(3, 2)
            .withProperties(Map.of("Number of columns", 5, "Number of rows", 3));

        layout.addBoolean("Left Grid", () -> selectedColumn < 3)
            .withWidget("Boolean Box")
            .withPosition(0,0);
        layout.addBoolean("Co-op Grid", () -> selectedColumn < 6 && selectedColumn > 2)
            .withWidget("Boolean Box")
            .withPosition(1,0);
        layout.addBoolean("Right Grid", () -> selectedColumn > 5)
            .withWidget("Boolean Box")
            .withPosition(2,0);

        layout.addBoolean("Left Sub-Column", () -> selectedColumn == 0 || selectedColumn == 3 || selectedColumn == 6)
            .withWidget("Boolean Box")
            .withPosition(0,1);
        layout.addBoolean("Middle Sub-Column", () -> selectedColumn == 1 || selectedColumn == 4 || selectedColumn == 7)
            .withWidget("Boolean Box")
            .withPosition(1,1);
        layout.addBoolean("Right Sub-Column", () -> selectedColumn == 2 || selectedColumn == 5 || selectedColumn == 8)
            .withWidget("Boolean Box")
            .withPosition(2,1);

        layout.addBoolean("High Node", () -> selectedLevel == NodeLevel.HIGH)
            .withWidget("Boolean Box")
            .withPosition(4, 2);
        layout.addBoolean("Mid Node", () -> selectedLevel == NodeLevel.MID)
            .withWidget("Boolean Box")
            .withPosition(4, 1);
        layout.addBoolean("Hybrid Node", () -> selectedLevel == NodeLevel.HYBRID)
            .withWidget("Boolean Box")
            .withPosition(4, 0);
    }

    @Override
    public void periodic() {
        // Send selected game piece
        SmartDashboard.putBoolean("Cube Selected", gamePiece == GamePiece.CUBE);

        // Send current node as text
        if (selectedRow == 0) {
            selectedLevel = NodeLevel.HYBRID;
        } else if (selectedRow == 1) {
            selectedLevel = NodeLevel.MID;
        } else {
            selectedLevel = NodeLevel.HIGH;
        }

        String text = "";
        switch (selectedLevel) {
            case HYBRID:
                text += "HYBRID";
                break;
            case MID:
                text += "MID";
                break;
            case HIGH:
                text += "HIGH";
                break;
        }

        text += ", ";
        if (selectedColumn < 3) {
            text += "LEFT";
        } else if (selectedColumn < 6) {
            text += "CO-OP";
        } else {
            text += "RIGHT";
        }

        text += " grid, ";

        if (selectedColumn == 1 || selectedColumn == 4 || selectedColumn == 7) {
            text += selectedLevel == NodeLevel.HYBRID ? "CENTER" : "CUBE";
        } else if (selectedColumn == 0 || selectedColumn == 3 || selectedColumn == 6) {
            text += "LEFT";
            text += selectedLevel == NodeLevel.HYBRID ? "" : " CONE";
        } else {
            text += "RIGHT";
            text += selectedLevel == NodeLevel.HYBRID ? "" : " CONE";
        }
        text += " node";
        SmartDashboard.putString("Selected Node", text);

        // Log state
        Logger.getInstance().recordOutput("ObjectiveTracker/GamePiece", gamePiece.toString());
        Logger.getInstance().recordOutput("ObjectiveTracker/SelectedRow", selectedRow);
        Logger.getInstance().recordOutput("ObjectiveTracker/SelectedLevel", selectedLevel.toString());
    }

    public synchronized SuperstructureState getDesiredSuperstructureState() {
        switch(selectedLevel) {
            case HIGH:
                return SuperstructureState.Preset.HIGH.getState();
            case MID:
                return SuperstructureState.Preset.MID.getState();
            case HYBRID:
                return SuperstructureState.Preset.LOW.getState();
        }
        return SuperstructureState.Preset.HOLDING.getState();
    }

    /** Shifts the selected node in the selector by one position. */
    public synchronized void shiftNode(Direction direction) {
        switch (direction) {
            case RIGHT:
                if (selectedRow < 8) {
                    selectedRow += 1;
                }
                break;

            case LEFT:
                if (selectedRow > 0) {
                    selectedRow -= 1;
                }
                break;

            case UP:
                switch (selectedLevel) {
                    case HYBRID:
                        break;
                    case MID: 
                        selectedLevel = NodeLevel.HYBRID;
                        selectedRow--;
                        break;
                    case HIGH:
                        selectedLevel = NodeLevel.MID;
                        selectedRow--;
                        break;
                }
                break;

            case DOWN:
                switch (selectedLevel) {
                    case HYBRID:
                        selectedLevel = NodeLevel.MID;
                        selectedRow++;
                        break;
                    case MID:
                        selectedLevel = NodeLevel.HIGH;
                        selectedRow++;
                        break;
                    case HIGH:
                        break;
                }
                break;
        }
    }

    /**
     * Command factory to shift the selected node in the selector by one position.
     */
    public Command shiftNodeCommand(Direction direction) {
        return new InstantCommand(() -> shiftNode(direction))
                .andThen(
                        Commands.waitSeconds(0.3),
                        Commands.repeatingSequence(
                                new InstantCommand(() -> shiftNode(direction)), new WaitCommand(0.1)))
                .ignoringDisable(true);
    }

    public static enum GamePiece {
        CUBE,
        CONE
    }

    public static enum NodeLevel {
        HYBRID,
        MID,
        HIGH
    }

    public static enum Substation {
        DOUBLE_LEFT,
        DOUBLE_RIGHT,
        SINGLE
    }

    public static enum Direction {
        LEFT,
        RIGHT,
        UP,
        DOWN
    }
}