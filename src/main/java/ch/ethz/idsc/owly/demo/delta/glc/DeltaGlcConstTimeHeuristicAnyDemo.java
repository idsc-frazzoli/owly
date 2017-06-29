// code by jl
package ch.ethz.idsc.owly.demo.delta.glc;

import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.glc.core.AnyPlannerInterface;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GlcNodes;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.gui.OwlyFrame;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.Trajectories;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

class DeltaGlcConstTimeHeuristicAnyDemo {
  public static void main(String[] args) throws Exception {
    // -- Quick Planner init
    RationalScalar quickResolution = (RationalScalar) RationalScalar.of(11, 1);
    Scalar timeScale = RealScalar.of(5);
    Scalar depthScale = RealScalar.of(10);
    Tensor partitionScale = Tensors.vector(10e5, 10e5);
    Scalar dtMax = RationalScalar.of(1, 6);
    int maxIter = 2000;
    TrajectoryPlanner quickTrajectoryPlanner = DeltaHelper.createGlc(RealScalar.of(-0.25), quickResolution);
    OwlyFrame quickOwlyFrame = Gui.start();
    quickOwlyFrame.configCoordinateOffset(33, 416);
    quickOwlyFrame.jFrame.setBounds(100, 100, 620, 475);
    // TODO FIX depthlimit needs to come from parameters
    Expand.maxDepth(quickTrajectoryPlanner, RealScalar.POSITIVE_INFINITY.number().intValue());
    quickOwlyFrame.setGlc(quickTrajectoryPlanner);
    Optional<GlcNode> optional = quickTrajectoryPlanner.getBest();
    if (optional.isPresent()) {
      List<StateTime> quickTrajectory = GlcNodes.getPathFromRootTo(optional.get());
      Trajectories.print(quickTrajectory);
    }
    // s --SLOWPLANNER
    RationalScalar resolution = (RationalScalar) RationalScalar.of(11, 1);
    AnyPlannerInterface anyTrajectoryPlanner = DeltaHelper.createGlcAny(RealScalar.of(-0.25), resolution);
    // GOALMANAGER
    OwlyFrame owlyFrame = Gui.start();
    owlyFrame.configCoordinateOffset(33, 416);
    owlyFrame.jFrame.setBounds(100, 100, 620, 475);
    Scalar planningTime = RealScalar.of(1);
    while (!anyTrajectoryPlanner.getBest().isPresent() && owlyFrame.jFrame.isVisible()) {
      // TODO JAN wierd RealScalar cast
      int expandIter = Expand.constTime(anyTrajectoryPlanner, (RealScalar) planningTime);
      owlyFrame.setGlc((TrajectoryPlanner) anyTrajectoryPlanner);
      if (expandIter < 1)
        break;
      Thread.sleep(1);
    }
  }
}
