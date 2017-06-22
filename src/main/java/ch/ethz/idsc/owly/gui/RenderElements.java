// code by jph
package ch.ethz.idsc.owly.gui;

import java.util.Collection;
import java.util.LinkedList;
import java.util.List;

import ch.ethz.idsc.owly.data.tree.StateCostNode;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.rrts.adapter.SampledTransitionRegionQuery;
import ch.ethz.idsc.owly.rrts.core.TransitionRegionQuery;

class RenderElements {
  final List<AbstractRender> list = new LinkedList<>();

  public RenderElements(TrajectoryPlanner trajectoryPlanner) {
    list.add(new EtaRender(trajectoryPlanner.getEta()));
    list.add(new DomainRender(trajectoryPlanner.getDomainMap(), trajectoryPlanner.getEta()));
    {
      TrajectoryRegionQuery trq = trajectoryPlanner.getObstacleQuery();
      if (trq instanceof SimpleTrajectoryRegionQuery) {
        SimpleTrajectoryRegionQuery strq = (SimpleTrajectoryRegionQuery) trq;
        list.add(new ObstacleRender(strq.getDiscoveredMembers()));
      }
    }
    list.add(new QueueRender(trajectoryPlanner.getQueue()));
    list.add(new TreeRender(trajectoryPlanner.getDomainMap().values()));
    list.add(new TrajectoryRender(trajectoryPlanner));
    {
      if (trajectoryPlanner instanceof OptimalAnyTrajectoryPlanner) {
        OptimalAnyTrajectoryPlanner optimalAnyTrajectoryPlanner = (OptimalAnyTrajectoryPlanner) trajectoryPlanner;
        list.add(new CandidatesRender(optimalAnyTrajectoryPlanner));
      }
    }
    {
      TrajectoryRegionQuery trq = trajectoryPlanner.getGoalQuery();
      if (trq instanceof SimpleTrajectoryRegionQuery) {
        SimpleTrajectoryRegionQuery strq = (SimpleTrajectoryRegionQuery) trq;
        list.add(new GoalRender(strq.getDiscoveredMembers()));
      }
    }
    list.add(new HudRender(trajectoryPlanner));
  }

  public RenderElements(Collection<? extends StateCostNode> collection, TransitionRegionQuery transitionRegionQuery) {
    if (transitionRegionQuery instanceof SampledTransitionRegionQuery) {
      SampledTransitionRegionQuery strq = (SampledTransitionRegionQuery) transitionRegionQuery;
      list.add(new ObstacleRender(strq.getDiscoveredMembers()));
    }
    list.add(new TreeRender(collection));
  }
}
