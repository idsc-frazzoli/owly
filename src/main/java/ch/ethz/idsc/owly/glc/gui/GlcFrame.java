// code by jph
package ch.ethz.idsc.owly.glc.gui;

import java.awt.BorderLayout;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JToolBar;
import javax.swing.WindowConstants;

public class GlcFrame {
  JFrame jFrame = new JFrame();
  public final GlcComponent glcComponent = new GlcComponent();

  public GlcFrame() {
    JPanel jPanel = new JPanel(new BorderLayout());
    jPanel.add(new JToolBar(), BorderLayout.NORTH);
    jPanel.add(glcComponent.jComponent, BorderLayout.CENTER);
    jFrame.setContentPane(jPanel);
    jFrame.setBounds(100, 50, 800, 800);
    jFrame.setVisible(true);
    jFrame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
  }
}
