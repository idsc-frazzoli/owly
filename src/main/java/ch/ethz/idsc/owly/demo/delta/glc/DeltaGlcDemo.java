// code by jl
package ch.ethz.idsc.owly.demo.delta.glc;

import java.util.List;

import ch.ethz.idsc.owly.data.Stopwatch;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.DebugUtils;
import ch.ethz.idsc.owly.glc.core.GlcExpand;
import ch.ethz.idsc.owly.glc.core.GlcNodes;
import ch.ethz.idsc.owly.gui.ani.OwlyFrame;
import ch.ethz.idsc.owly.gui.ani.OwlyGui;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

enum DeltaGlcDemo {
  ;
  public static void main(String[] args) throws Exception {
    RationalScalar resolution = (RationalScalar) RationalScalar.of(14, 1);
    Tensor partitionScale = Tensors.vector(60, 60);
    TrajectoryPlannerContainer trajectoryPlannerContainer = DeltaHelper.createGlc(RealScalar.of(-0.02), resolution, partitionScale);
    OwlyFrame owlyFrame = OwlyGui.start();
    owlyFrame.configCoordinateOffset(33, 416);
    owlyFrame.jFrame.setBounds(100, 100, 620, 475);
    Stopwatch timer = Stopwatch.started();
    int iters = 0;
    while (!trajectoryPlannerContainer.getTrajectoryPlanner().getBest().isPresent() && owlyFrame.jFrame.isVisible()) {
      iters = iters + GlcExpand.maxSteps(trajectoryPlannerContainer.getTrajectoryPlanner(), 30, trajectoryPlannerContainer.getParameters().getDepthLimit());
      // owlyFrame.setGlc(trajectoryPlannerContainer.getTrajectoryPlanner());
      Thread.sleep(1);
      DebugUtils.heuristicConsistencyCheck(trajectoryPlannerContainer.getTrajectoryPlanner());
      if (trajectoryPlannerContainer.getTrajectoryPlanner().getQueue().isEmpty())
        break;
    }
    System.out.println("Planning took " + iters + " in " + timer.display_seconds() + "s");
    owlyFrame.setGlc(trajectoryPlannerContainer.getTrajectoryPlanner());
    System.out.println("Goal has cost: " + trajectoryPlannerContainer.getTrajectoryPlanner().getBest().get().costFromRoot());
    List<StateTime> trajectory = GlcNodes.getPathFromRootTo(trajectoryPlannerContainer.getTrajectoryPlanner().getBest().get());
    StateTimeTrajectories.print(trajectory);
  }
}
