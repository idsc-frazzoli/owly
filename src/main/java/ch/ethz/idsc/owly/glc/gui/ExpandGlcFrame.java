// code by jph
package ch.ethz.idsc.owly.glc.gui;

import java.awt.BorderLayout;
import java.awt.FlowLayout;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JToolBar;
import javax.swing.WindowConstants;

import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;

public class ExpandGlcFrame {
  JFrame jFrame = new JFrame();
  final GlcComponent glcComponent = new GlcComponent();

  public ExpandGlcFrame(TrajectoryPlanner trajectoryPlanner) {
    glcComponent.trajectoryPlanner = trajectoryPlanner;
    JPanel jPanel = new JPanel(new BorderLayout());
    {
      JToolBar jToolBar = new JToolBar();
      jToolBar.setFloatable(false);
      jToolBar.setLayout(new FlowLayout(FlowLayout.LEFT, 5, 1));
      {
        JButton jButton = new JButton("expand");
        jButton.addActionListener(event -> {
          Expand.maxSteps(trajectoryPlanner, 1);
          glcComponent.jComponent.repaint();
        });
        jToolBar.add(jButton);
      }
      jPanel.add(jToolBar, BorderLayout.NORTH);
    }
    jPanel.add(glcComponent.jComponent, BorderLayout.CENTER);
    jFrame.setContentPane(jPanel);
    jFrame.setBounds(100, 50, 800, 800);
    jFrame.setVisible(true);
    jFrame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
  }
}
