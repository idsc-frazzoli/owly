// code by jph
package ch.ethz.idsc.owly.gui.ren;

import java.util.Collection;
import java.util.LinkedList;
import java.util.List;

import ch.ethz.idsc.owly.data.tree.StateCostNode;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.rrts.adapter.SampledTransitionRegionQuery;
import ch.ethz.idsc.owly.rrts.core.TransitionRegionQuery;

public enum RenderElements {
  ;
  public static Collection<RenderInterface> create(TrajectoryPlanner trajectoryPlanner) {
    List<RenderInterface> list = new LinkedList<>();
    list.add(GridRender.INSTANCE);
    list.add(new EtaRender(trajectoryPlanner.getEta()));
    list.add(new DomainRender(trajectoryPlanner.getDomainMap(), trajectoryPlanner.getEta()));
    {
      TrajectoryRegionQuery trq = trajectoryPlanner.getObstacleQuery();
      if (trq instanceof SimpleTrajectoryRegionQuery) {
        SimpleTrajectoryRegionQuery strq = (SimpleTrajectoryRegionQuery) trq;
        list.add(new ObstacleRender(strq.getSparseDiscoveredMembers()));
      }
    }
    list.add(new QueueRender(trajectoryPlanner.getQueue()));
    list.add(new TreeRender(trajectoryPlanner.getDomainMap().values()));
    list.add(TrajectoryRender.of(trajectoryPlanner));
    {
      if (trajectoryPlanner instanceof OptimalAnyTrajectoryPlanner) {
        OptimalAnyTrajectoryPlanner optimalAnyTrajectoryPlanner = (OptimalAnyTrajectoryPlanner) trajectoryPlanner;
        list.add(new CandidatesRender(optimalAnyTrajectoryPlanner));
      }
    }
    {
      TrajectoryRegionQuery trq = trajectoryPlanner.getGoalInterface();
      if (trq instanceof SimpleTrajectoryRegionQuery) {
        SimpleTrajectoryRegionQuery strq = (SimpleTrajectoryRegionQuery) trq;
        list.add(new GoalRender(strq.getSparseDiscoveredMembers()));
      }
      if (trq instanceof SimpleTrajectoryRegionQuery) {
        SimpleTrajectoryRegionQuery strq = (SimpleTrajectoryRegionQuery) trq;
        list.add(new GoalRender(strq.getSparseDiscoveredMembers()));
      }
    }
    list.add(new HudRender(trajectoryPlanner));
    return list;
  }

  public static Collection<RenderInterface> create( //
      Collection<? extends StateCostNode> collection, TransitionRegionQuery transitionRegionQuery) {
    List<RenderInterface> list = new LinkedList<>();
    list.add(GridRender.INSTANCE);
    if (transitionRegionQuery instanceof SampledTransitionRegionQuery) {
      SampledTransitionRegionQuery strq = (SampledTransitionRegionQuery) transitionRegionQuery;
      list.add(new ObstacleRender(strq.getDiscoveredMembers()));
    }
    list.add(new TreeRender(collection));
    return list;
  }
}
