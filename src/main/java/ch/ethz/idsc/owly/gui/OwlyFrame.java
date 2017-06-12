// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.image.BufferedImage;
import java.io.Serializable;
import java.util.Collection;

import javax.imageio.ImageIO;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JToolBar;
import javax.swing.WindowConstants;

import ch.ethz.idsc.owly.data.tree.Nodes;
import ch.ethz.idsc.owly.demo.util.UserHome;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.rrts.core.RrtsNode;
import ch.ethz.idsc.owly.rrts.core.TransitionRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.io.Serialization;

public class OwlyFrame {
  public final JFrame jFrame = new JFrame();
  private final OwlyComponent owlyComponent = new OwlyComponent();
  private final JLabel jLabel = new JLabel();

  public OwlyFrame() {
    JPanel jPanel = new JPanel(new BorderLayout());
    {
      JToolBar jToolBar = new JToolBar();
      jToolBar.setFloatable(false);
      JButton jButton = new JButton("save2png");
      jButton.setToolTipText("file is created in Pictures/...");
      jButton.addActionListener(new ActionListener() {
        @Override
        public void actionPerformed(ActionEvent actionEvent) {
          try {
            BufferedImage bufferedImage = offscreen();
            ImageIO.write(bufferedImage, "PNG", UserHome.Pictures("owly_" + System.currentTimeMillis() + ".png"));
          } catch (Exception exception) {
            exception.printStackTrace();
          }
        }
      });
      jToolBar.add(jButton);
      jPanel.add(jToolBar, BorderLayout.NORTH);
    }
    jPanel.add(owlyComponent.jComponent, BorderLayout.CENTER);
    jPanel.add(jLabel, BorderLayout.SOUTH);
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
      jLabel.setText(trajectoryPlanner.infoString());
      owlyComponent.jComponent.repaint();
    } catch (Exception exception) {
      exception.printStackTrace();
    }
  }

  @SuppressWarnings("unchecked")
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

  public BufferedImage offscreen() {
    Dimension dimension = owlyComponent.jComponent.getSize();
    BufferedImage myBufferedImage = new BufferedImage(dimension.width, dimension.height, BufferedImage.TYPE_INT_ARGB);
    owlyComponent.render(myBufferedImage.createGraphics(), dimension);
    return myBufferedImage;
  }
}
