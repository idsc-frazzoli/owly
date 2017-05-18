// code by jph
package ch.ethz.idsc.owly.gui;

import java.util.LinkedList;
import java.util.List;

import ch.ethz.idsc.owly.data.tree.Nodes;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.rrts.adapter.SampledTransitionRegionQuery;
import ch.ethz.idsc.owly.rrts.core.RrtsNode;
import ch.ethz.idsc.owly.rrts.core.TransitionRegionQuery;

class RenderElements {
  List<AbstractRender> list = new LinkedList<>();

  public RenderElements(RrtsNode root, TransitionRegionQuery transitionRegionQuery) {
    if (transitionRegionQuery instanceof SampledTransitionRegionQuery) {
      SampledTransitionRegionQuery strq = (SampledTransitionRegionQuery) transitionRegionQuery;
      list.add(new ObstacleRender(strq.getDiscoveredMembers()));
    }
    list.add(new TreeRender(Nodes.ofSubtree(root)));
  }

  public RenderElements(TrajectoryPlanner trajectoryPlanner) {
    list.add(new DomainRender(trajectoryPlanner.getEta()));
    {
      TrajectoryRegionQuery trq = trajectoryPlanner.getObstacleQuery();
      if (trq instanceof SimpleTrajectoryRegionQuery) {
        SimpleTrajectoryRegionQuery strq = (SimpleTrajectoryRegionQuery) trq;
        list.add(new ObstacleRender(strq.getDiscoveredMembers()));
      }
    }
    list.add(new QueueRender(trajectoryPlanner.getQueue()));
    list.add(new TreeRender(trajectoryPlanner.getNodes()));
    list.add(new TrajectoryRender(trajectoryPlanner));
    {
      TrajectoryRegionQuery trq = trajectoryPlanner.getGoalQuery();
      if (trq instanceof SimpleTrajectoryRegionQuery) {
        SimpleTrajectoryRegionQuery strq = (SimpleTrajectoryRegionQuery) trq;
        list.add(new GoalRender(strq.getDiscoveredMembers()));
      }
    }
  }
}
