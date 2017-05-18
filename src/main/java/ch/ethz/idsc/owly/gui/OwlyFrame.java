// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.BorderLayout;
import java.io.Serializable;
import java.util.Collection;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JToolBar;
import javax.swing.WindowConstants;

import ch.ethz.idsc.owly.data.tree.Nodes;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.rrts.core.RrtsNode;
import ch.ethz.idsc.owly.rrts.core.TransitionRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.io.Serialization;

public class OwlyFrame {
  public final JFrame jFrame = new JFrame();
  private final OwlyComponent owlyComponent = new OwlyComponent();

  public OwlyFrame() {
    JPanel jPanel = new JPanel(new BorderLayout());
    jPanel.add(new JToolBar(), BorderLayout.NORTH);
    jPanel.add(owlyComponent.jComponent, BorderLayout.CENTER);
    jFrame.setContentPane(jPanel);
    jFrame.setBounds(100, 50, 800, 800);
    jFrame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
  }

  public void configCoordinateOffset(int px, int py) {
    owlyComponent.model2pixel.set(RealScalar.of(px), 0, 2);
    owlyComponent.model2pixel.set(RealScalar.of(py), 1, 2);
  }

  public void setGlc(TrajectoryPlanner trajectoryPlanner) {
    try {
      owlyComponent.renderElements = new RenderElements(Serialization.copy(trajectoryPlanner));
      owlyComponent.jComponent.repaint();
    } catch (Exception exception) {
      exception.printStackTrace();
    }
  }

  public void setRrts(RrtsNode root, TransitionRegionQuery transitionRegionQuery) {
    try {
      Collection<RrtsNode> nodes = Nodes.ofSubtree(root);
      Collection<RrtsNode> collection = (Collection<RrtsNode>) Serialization.copy((Serializable) nodes);
      owlyComponent.renderElements = new RenderElements(collection, Serialization.copy(transitionRegionQuery));
      owlyComponent.jComponent.repaint();
    } catch (Exception exception) {
      exception.printStackTrace();
    }
  }

  @Deprecated
  public void repaint() { // see Se2rExpandDemo for how to render updates
    owlyComponent.jComponent.repaint();
  }
}
