// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.util.Collection;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.DecimalScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.Round;

class HudLayer extends AbstractLayer {
  TrajectoryPlanner trajectoryPlanner;

  HudLayer(OwlyComponent glcComponent) {
    super(glcComponent);
    MouseListener mouseListener = new MouseAdapter() {
      @Override
      public void mousePressed(MouseEvent mouseEvent) {
        Tensor location = glcComponent.toTensor(mouseEvent.getPoint());
        location = location.extract(0, 2);
        System.out.println(location.map(Round.toMultipleOf(DecimalScalar.of(0.001))) + ",");
      }
    };
    glcComponent.jComponent.addMouseListener(mouseListener);
  }

  void setTrajectoryPlanner(TrajectoryPlanner trajectoryPlanner) {
    this.trajectoryPlanner = trajectoryPlanner;
  }

  void render(Graphics2D graphics) {
    // ---
    graphics.setColor(new Color(0, 0, 0, 64));
    {
      Collection<GlcNode> collection = trajectoryPlanner.getDomainMap().values();
      graphics.drawString("nodes:" + collection.size(), 0, 10);
    }
    {
      TrajectoryRegionQuery trq = trajectoryPlanner.getObstacleQuery();
      if (trq instanceof SimpleTrajectoryRegionQuery) {
        SimpleTrajectoryRegionQuery strq = (SimpleTrajectoryRegionQuery) trq;
        Collection<StateTime> collection = strq.getDiscoveredMembers();
        graphics.drawString("obstacles:" + collection.size(), 0, 20);
        // LruCache.create(maxSize)
      }
      // Collection<Node> collection = trajectoryPlanner.getNodes();
      // graphics.drawString("nodes:" + collection.size(), 0, 10);
    }
    {
      graphics.drawString("replacements:" + trajectoryPlanner.replaceCount(), 0, 30);
    }
  }
}
