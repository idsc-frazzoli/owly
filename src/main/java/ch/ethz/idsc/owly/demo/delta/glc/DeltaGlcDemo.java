// code by jl
package ch.ethz.idsc.owly.demo.delta.glc;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.adapter.TrajectoryPlannerContainer;
import ch.ethz.idsc.owly.glc.core.DebugUtils;
import ch.ethz.idsc.owly.glc.core.GlcExpand;
import ch.ethz.idsc.owly.glc.core.GlcNodes;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.gui.OwlyFrame;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

enum DeltaGlcDemo {
  ;
  public static void main(String[] args) throws Exception {
    RationalScalar resolution = (RationalScalar) RationalScalar.of(10, 1);
    Tensor partitionScale = Tensors.vector(2e26, 2e26);
    TrajectoryPlannerContainer trajectoryPlannerContainer = DeltaHelper.createGlc(RealScalar.of(-0.5), resolution, partitionScale);
    OwlyFrame owlyFrame = Gui.start();
    owlyFrame.configCoordinateOffset(33, 416);
    owlyFrame.jFrame.setBounds(100, 100, 620, 475);
    while (!trajectoryPlannerContainer.getTrajectoryPlanner().getBest().isPresent() && owlyFrame.jFrame.isVisible()) {
      GlcExpand.maxSteps(trajectoryPlannerContainer.getTrajectoryPlanner(), 30, trajectoryPlannerContainer.getParameters().getDepthLimit());
      owlyFrame.setGlc(trajectoryPlannerContainer.getTrajectoryPlanner());
      Thread.sleep(1);
      DebugUtils.heuristicConsistencyCheck(trajectoryPlannerContainer.getTrajectoryPlanner());
      if (trajectoryPlannerContainer.getTrajectoryPlanner().getQueue().isEmpty())
        break;
    }
    List<StateTime> trajectory = GlcNodes.getPathFromRootTo(trajectoryPlannerContainer.getTrajectoryPlanner().getBest().get());
    StateTimeTrajectories.print(trajectory);
  }
}
