// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.image.BufferedImage;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import javax.imageio.ImageIO;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.JToggleButton;
import javax.swing.JToolBar;
import javax.swing.WindowConstants;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import ch.ethz.idsc.owly.data.tree.Nodes;
import ch.ethz.idsc.owly.demo.util.UserHome;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectorySample;
import ch.ethz.idsc.owly.gui.misc.ImageRegionRender;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.owly.rrts.core.RrtsNode;
import ch.ethz.idsc.owly.rrts.core.TransitionRegionQuery;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.io.Serialization;

public class OwlyFrame {
  public final JFrame jFrame = new JFrame();
  private final OwlyComponent owlyComponent = new OwlyComponent();
  private final JLabel jLabel = new JLabel();
  private boolean replay = false;
  private int replayIndex = 0;
  private final List<TrajectoryPlanner> backup = new ArrayList<>();
  private final JSlider jSlider = new JSlider();

  public OwlyFrame() {
    JPanel jPanel = new JPanel(new BorderLayout());
    {
      JToolBar jToolBar = new JToolBar();
      jToolBar.setFloatable(false);
      {
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
      }
      {
        JButton jButton = new JButton("Reset View");
        jButton.setToolTipText("Resets the Crop and Zoom of the Window");
        jButton.addActionListener(new ActionListener() {
          @Override
          public void actionPerformed(ActionEvent actionEvent) {
            owlyComponent.reset_model2pixel();
            repaint(replayIndex);
          }
        });
        jToolBar.add(jButton);
      }
      {
        JToggleButton jToggleButton = new JToggleButton("Replay");
        jToggleButton.setToolTipText("stops LiveFeed and goes to Replaymode");
        jToggleButton.addActionListener(new ActionListener() {
          @Override
          public void actionPerformed(ActionEvent actionEvent) {
            replay = jToggleButton.isSelected();
          }
        });
        jToolBar.add(jToggleButton);
      }
      {
        JButton jButton = new JButton("<<");
        jButton.setToolTipText("Replay: 1 Step back");
        jButton.addActionListener(new ActionListener() {
          @Override
          public void actionPerformed(ActionEvent actionEvent) {
            if (replayIndex > 0) {
              replayIndex = replayIndex - 1;
            } else {
              replayIndex = 0;
              System.err.println("GUI: Already displaying first Planningstep");
            }
            jSlider.setValue(replayIndex);
          }
        });
        jToolBar.add(jButton);
      }
      {
        JButton jButton = new JButton(">>");
        jButton.setToolTipText("Replay: 1 Step forward");
        jButton.addActionListener(new ActionListener() {
          @Override
          public void actionPerformed(ActionEvent actionEvent) {
            if (replayIndex < backup.size() - 1) {
              replayIndex = replayIndex + 1;
            } else {
              replayIndex = backup.size() - 1;
              System.err.println("GUI: Already displaying latest Planningstep");
            }
            jSlider.setValue(replayIndex);
          }
        });
        jToolBar.add(jButton);
      }
      jPanel.add(jToolBar, BorderLayout.NORTH);
      {
        jSlider.setOpaque(false);
        jSlider.addChangeListener(new ChangeListener() {
          @Override
          public void stateChanged(ChangeEvent e) {
            replayIndex = jSlider.getValue();
            repaint(replayIndex);
          }
        });
        jToolBar.add(jSlider);
      }
    }
    jPanel.add(owlyComponent.jComponent, BorderLayout.CENTER);
    jPanel.add(jLabel, BorderLayout.SOUTH);
    jFrame.setContentPane(jPanel);
    jFrame.setBounds(100, 50, 800, 800);
    jFrame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
  }

  public void configCoordinateOffset(int px, int py) {
    owlyComponent.setOffset(Tensors.vector(px, py));
  }

  public void setGlc(TrajectoryPlanner trajectoryPlanner) {
    try {
      backup.add(Serialization.copy(trajectoryPlanner));
      jSlider.setMaximum(backup.size() - 1);
    } catch (Exception e) {
      // ---
      e.printStackTrace();
    }
    if (!replay) { // live feed
      replayIndex = backup.size() - 1;
      jSlider.setValue(replayIndex);
    }
  }

  private void repaint(int index) {
    if (0 <= index && index < backup.size())
      try {
        owlyComponent.renderElements = new RenderElements(backup.get(index));
        jLabel.setText(backup.get(index).infoString());
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

  public void addBackground(ImageRegion imageRegion) {
    addBackground(new ImageRegionRender(imageRegion));
  }

  public void addBackground(RenderInterface renderInterface) {
    owlyComponent.addDrawable(renderInterface);
  }

  public void addTrajectory(List<TrajectorySample> trajectory) {
    TrajectoryRender trajectoryRenderer = new TrajectoryRender(null);
    trajectoryRenderer.setTrajectory(trajectory);
    owlyComponent.addDrawable(trajectoryRenderer);
  }
}
