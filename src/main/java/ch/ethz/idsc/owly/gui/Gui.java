// code by jph
package ch.ethz.idsc.owly.gui;

import java.util.Collection;

import ch.ethz.idsc.owly.data.tree.Nodes;
import ch.ethz.idsc.owly.data.tree.StateCostNode;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.rrts.adapter.SampledTransitionRegionQuery;
import ch.ethz.idsc.owly.rrts.core.RrtsNode;
import ch.ethz.idsc.owly.rrts.core.TransitionRegionQuery;

public enum Gui {
  ;
  // ---
  public static OwlyFrame glc(TrajectoryPlanner trajectoryPlanner) {
    OwlyFrame owlyFrame = new OwlyFrame();
    OwlyComponent c = owlyFrame.owlyComponent;
    {
      DomainLayer d = new DomainLayer(c);
      d.setEta(trajectoryPlanner.getEta());
      c.layers.add(d);
    }
    {
      TrajectoryRegionQuery trq = trajectoryPlanner.getObstacleQuery();
      if (trq instanceof SimpleTrajectoryRegionQuery) {
        SimpleTrajectoryRegionQuery strq = (SimpleTrajectoryRegionQuery) trq;
        ObstacleLayer l = new ObstacleLayer(c);
        l.setDiscoveredMembers(strq.getDiscoveredMembers());
        c.layers.add(l);
      }
    }
    {
      QueueLayer l = new QueueLayer(c);
      l.setCollection(trajectoryPlanner.getQueue());
      c.layers.add(l);
    }
    {
      TreeLayer l = new TreeLayer(c);
      l.setCollection(trajectoryPlanner.getNodes());
      c.layers.add(l);
    }
    {
      TrajectoryLayer l = new TrajectoryLayer(c);
      l.setTrajectoryPlanner(trajectoryPlanner);
      c.layers.add(l);
    }
    {
      TrajectoryRegionQuery trq = trajectoryPlanner.getGoalQuery();
      if (trq instanceof SimpleTrajectoryRegionQuery) {
        SimpleTrajectoryRegionQuery strq = (SimpleTrajectoryRegionQuery) trq;
        GoalLayer l = new GoalLayer(c);
        l.setDiscoveredMembers(strq.getDiscoveredMembers());
        c.layers.add(l);
      }
    }
    {
      HudLayer hudLayer = new HudLayer(c);
      hudLayer.setTrajectoryPlanner(trajectoryPlanner);
      c.layers.add(hudLayer);
    }
    owlyFrame.jFrame.setVisible(true);
    return owlyFrame;
  }

  public static void rrts(RrtsNode root, TransitionRegionQuery transitionRegionQuery) {
    OwlyFrame owlyFrame = new OwlyFrame();
    OwlyComponent c = owlyFrame.owlyComponent;
    {
      if (transitionRegionQuery instanceof SampledTransitionRegionQuery) {
        SampledTransitionRegionQuery strq = (SampledTransitionRegionQuery) transitionRegionQuery;
        ObstacleLayer l = new ObstacleLayer(c);
        l.setDiscoveredMembers(strq.getDiscoveredMembers());
        c.layers.add(l);
      }
    }
    {
      TreeLayer l = new TreeLayer(c);
      Collection<StateCostNode> nodes = Nodes.ofSubtree(root);
      l.setCollection(nodes);
      c.layers.add(l);
    }
    owlyFrame.jFrame.setVisible(true);
  }
}
