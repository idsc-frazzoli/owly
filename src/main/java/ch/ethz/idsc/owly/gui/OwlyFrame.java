// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.BorderLayout;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JToolBar;
import javax.swing.WindowConstants;

public class OwlyFrame {
  JFrame jFrame = new JFrame();
  public final OwlyComponent owlyComponent = new OwlyComponent();

  public OwlyFrame() {
    JPanel jPanel = new JPanel(new BorderLayout());
    jPanel.add(new JToolBar(), BorderLayout.NORTH);
    jPanel.add(owlyComponent.jComponent, BorderLayout.CENTER);
    jFrame.setContentPane(jPanel);
    jFrame.setBounds(100, 50, 800, 800);
    jFrame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
  }

  public void repaint() {
    owlyComponent.jComponent.repaint();
  }
}
