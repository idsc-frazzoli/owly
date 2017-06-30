// code by jl
package ch.ethz.idsc.owly.demo.delta.glc;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.demo.delta.DeltaStateSpaceModel;
import ch.ethz.idsc.owly.demo.delta.DeltaTrajectoryGoalManager;
import ch.ethz.idsc.owly.demo.delta.ImageGradient;
import ch.ethz.idsc.owly.demo.util.Images;
import ch.ethz.idsc.owly.demo.util.Resources;
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
import ch.ethz.idsc.tensor.io.Import;

class DeltaGlcConstTimeHeuristicAnyDemo {
  public static void main(String[] args) throws Exception {
    // -- Quick Planner init
    RationalScalar quickResolution = (RationalScalar) RationalScalar.of(11, 1);
    TrajectoryPlanner quickTrajectoryPlanner = DeltaHelper.createGlc(RealScalar.of(-0.25), quickResolution);
    OwlyFrame quickOwlyFrame = Gui.start();
    quickOwlyFrame.configCoordinateOffset(33, 416);
    quickOwlyFrame.jFrame.setBounds(100, 100, 620, 475);
    // TODO FIX depthlimit needs to come from parameters
    Expand.maxDepth(quickTrajectoryPlanner, RealScalar.POSITIVE_INFINITY.number().intValue());
    quickOwlyFrame.setGlc(quickTrajectoryPlanner);
    Optional<GlcNode> optional = quickTrajectoryPlanner.getBest();
    List<StateTime> quickTrajectory = null;
    if (optional.isPresent()) {
      quickTrajectory = GlcNodes.getPathFromRootTo(optional.get());
      Trajectories.print(quickTrajectory);
    } else {
      throw new RuntimeException();
    }
    // s --SLOWPLANNER
    RationalScalar resolution = (RationalScalar) RationalScalar.of(11, 1);
    AnyPlannerInterface slowTrajectoryPlanner = DeltaHelper.createGlcAny(RealScalar.of(-0.25), resolution);
    // GOALMANAGER
    // TODO: needs to be removed from main
    Tensor range = Tensors.vector(9, 6.5);
    ImageGradient ipr = new ImageGradient( //
        Images.displayOrientation(Import.of(Resources.fileFromRepository("/io/delta_uxy.png")).get(Tensor.ALL, Tensor.ALL, 0)), //
        range, RealScalar.of(-.5)); // -.25 .5
    DeltaStateSpaceModel stateSpaceModel = new DeltaStateSpaceModel(ipr);
    Scalar maxInput = RealScalar.ONE;
    Scalar maxSpeed = maxInput.add(ipr.maxNorm());
    Iterator<StateTime> iterator = quickTrajectory.iterator();
    List<Tensor> quickPath = new ArrayList<>();
    while (iterator.hasNext())
      quickPath.add(iterator.next().x());
    DeltaTrajectoryGoalManager trajectoryGoalManager = new DeltaTrajectoryGoalManager(//
        quickPath, Tensors.vector(.3, .3), maxSpeed);
    slowTrajectoryPlanner.changeToGoal(trajectoryGoalManager);
    OwlyFrame owlyFrame = Gui.start();
    owlyFrame.configCoordinateOffset(33, 416);
    owlyFrame.jFrame.setBounds(100, 100, 620, 475);
    Scalar planningTime = RealScalar.of(1);
    while (owlyFrame.jFrame.isVisible()) {
      // TODO JAN wierd RealScalar cast
      int expandIter = Expand.constTime(slowTrajectoryPlanner, (RealScalar) planningTime);
      owlyFrame.setGlc((TrajectoryPlanner) slowTrajectoryPlanner);
      if (expandIter < 1)
        break;
      Thread.sleep(1);
    }
  }
}
