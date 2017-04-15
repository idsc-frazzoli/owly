// code by jph
package ch.ethz.idsc.owly.glc.gui;

import javax.swing.JFrame;
import javax.swing.WindowConstants;

public class GlcFrame {
  JFrame jFrame = new JFrame();
  public final GlcComponent glcComponent = new GlcComponent();

  public GlcFrame() {
    jFrame.setContentPane(glcComponent.jComponent);
    jFrame.setBounds(100, 50, 800, 800);
    jFrame.setVisible(true);
    jFrame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
  }
}
