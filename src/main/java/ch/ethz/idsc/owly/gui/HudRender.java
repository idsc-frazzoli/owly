// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.Color;
import java.awt.Graphics2D;
import java.util.Collection;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;

class HudRender implements AbstractRender {
  private final TrajectoryPlanner trajectoryPlanner;

  HudRender(TrajectoryPlanner trajectoryPlanner) {
    this.trajectoryPlanner = trajectoryPlanner;
  }

  @Override
  public void render(OwlyLayer abstractLayer, Graphics2D graphics) {
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
