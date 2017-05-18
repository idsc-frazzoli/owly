// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.BorderLayout;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JToolBar;
import javax.swing.WindowConstants;

import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.tensor.io.Serialization;

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

  public void setGlc(TrajectoryPlanner trajectoryPlanner) {
    try {
      owlyComponent.renderElements = new RenderElements(Serialization.copy(trajectoryPlanner));
      owlyComponent.jComponent.repaint();
    } catch (Exception exception) {
      exception.printStackTrace();
    }
  }

  public void repaint() {
    owlyComponent.jComponent.repaint();
  }
}
