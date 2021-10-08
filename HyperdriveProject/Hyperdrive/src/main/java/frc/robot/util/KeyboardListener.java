// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.awt.KeyEventDispatcher;
import java.awt.KeyboardFocusManager;
import java.awt.event.KeyEvent;
import java.awt.Dimension;
import java.awt.HeadlessException;

import javax.swing.JFrame;
import javax.swing.JLabel;

import edu.wpi.first.wpilibj.DriverStation;

/** Listens to the keys. */
public class KeyboardListener {
    private static boolean
        isWPressed,
        isAPressed,
        isSPressed,
        isDPressed;

    public static boolean getWPressed() {
        return isWPressed;
    }

    public static boolean getAPressed() {
        return isAPressed;
    }

    public static boolean getSPressed() {
        return isSPressed;
    }

    public static boolean getDPressed() {
        return isDPressed;
    }

    public static void start() {
        JFrame inputArea;
        try {
            inputArea = new JFrame();
        } catch(HeadlessException ex) {
            DriverStation.reportError("KeyboardListener could not start becuase the system does not support it.", true);
            return;
        }
        inputArea.setPreferredSize(new Dimension(100, 100));
        inputArea.add(new JLabel("Focus this window to enter keyboard input"));
        inputArea.setVisible(true);

        KeyboardFocusManager.getCurrentKeyboardFocusManager().addKeyEventDispatcher(new KeyEventDispatcher() {
            @Override
            public boolean dispatchKeyEvent(KeyEvent event) {
                switch(event.getID()) {
                    case KeyEvent.KEY_PRESSED: {
                        //which key was pressed?
                        switch(event.getKeyCode()) {
                            case KeyEvent.VK_W: {
                                isWPressed = true;
                            }
                            break;

                            case KeyEvent.VK_A: {
                                isAPressed = true;
                            }
                            break;

                            case KeyEvent.VK_S: {
                                isSPressed = true;
                            }
                            break;

                            case KeyEvent.VK_D: {
                                isDPressed = true;
                            }

                            default: //default for cases that arent one of the w, a, s, or d keys
                                break;
                        }
                    }
                    break;

                    case KeyEvent.KEY_RELEASED: {
                        //which key was released?
                        switch(event.getKeyCode()) {
                            case KeyEvent.VK_W: {
                                isWPressed = false;
                            }
                            break;

                            case KeyEvent.VK_A: {
                                isAPressed = false;
                            }
                            break;

                            case KeyEvent.VK_S: {
                                isSPressed = false;
                            }
                            break;

                            case KeyEvent.VK_D: {
                                isDPressed = false;
                            }
                            break;

                            default: //default for cases that arent any one of the w, a, s, or d keys
                                break;
                        }
                    }
                    break;

                    default: //default for cases that aren't key pressed or key released
                        break;
                }

                return false;
            }
        });

        DriverStation.reportWarning("Keyboard Listener Started.", false);
    }
}
