package JavaHWs;

/**
 *  @author Tyler Bryk
 *  I pledge my honor that I have abided by the Stevens Honor System
 */

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;

public class CalculatorBryk extends JFrame {
    private String[] buttonChar = {"7", "8", "9", "÷", "4", "5", "6", "x", "1", "2", "3", "-", "Clear", "0", "=", "+"};
    private JTextArea display = new JTextArea(" ");
    private JButton[] buttons = new JButton[16];
    
    public CalculatorBryk() {
        super("Calculator");
        setSize(700,700);
        Container content = getContentPane();
        Font displayFont = new Font("Helvetica", Font.PLAIN, 100);
        Font buttonFont = new Font("Helvetica", Font.PLAIN, 40);
        display.setFont(displayFont);
        display.setMargin(new Insets(10,0,0,0));
        
        JPanel pane = new JPanel();
        pane.setBackground(Color.GRAY);
        pane.setLayout(new GridLayout(4,4));
        for (int i = 0; i < 16; i++) {
            buttons[i] = new JButton(buttonChar[i]);
            pane.add(buttons[i]);
            buttons[i].addActionListener(new MyActionListener());
            buttons[i].setFont(buttonFont);
        }

        content.add(BorderLayout.CENTER, pane);
        content.add(BorderLayout.NORTH, display);
        setDefaultCloseOperation(EXIT_ON_CLOSE);
        setVisible(true);
    }
    
    class MyActionListener implements ActionListener{
        public void actionPerformed(ActionEvent e) {
            if(e.getSource() == buttons[0]) { display.append(buttonChar[0]+""); }
            else if(e.getSource() == buttons[1]) { display.append(buttonChar[1]+""); }
            else if(e.getSource() == buttons[2]) { display.append(buttonChar[2]+""); }
            else if(e.getSource() == buttons[3]) { display.append(buttonChar[3]+""); }
            else if(e.getSource() == buttons[4]) { display.append(buttonChar[4]+""); }
            else if(e.getSource() == buttons[5]) { display.append(buttonChar[5]+""); }
            else if(e.getSource() == buttons[6]) { display.append(buttonChar[6]+""); }
            else if(e.getSource() == buttons[7]) { display.append(buttonChar[7]+""); }
            else if(e.getSource() == buttons[8]) { display.append(buttonChar[8]+""); }
            else if(e.getSource() == buttons[9]) { display.append(buttonChar[9]+""); }
            else if(e.getSource() == buttons[10]) { display.append(buttonChar[10]+""); }
            else if(e.getSource() == buttons[11]) { display.append(buttonChar[11]+""); }
            else if(e.getSource() == buttons[12]) { display.setText(" "); }
            else if(e.getSource() == buttons[13]) { display.append(buttonChar[13]+""); }
            else if(e.getSource() == buttons[14]) { display.append(buttonChar[14]+""); }
            else if(e.getSource() == buttons[15]) { display.append(buttonChar[15]+""); }
        }
    }
    
    public static void main(String[] args) {
        new CalculatorBryk();
    }
}