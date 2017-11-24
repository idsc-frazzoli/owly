// code by jl
package ch.ethz.idsc.owly.demo.se2.any;

import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owl.glc.adapter.GlcExpand;
import ch.ethz.idsc.owl.glc.adapter.GlcNodes;
import ch.ethz.idsc.owl.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owl.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owl.glc.any.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.owl.glc.core.GlcNode;
import ch.ethz.idsc.owl.glc.par.Parameters;
import ch.ethz.idsc.owl.gui.ani.OwlyFrame;
import ch.ethz.idsc.owl.gui.ani.OwlyGui;
import ch.ethz.idsc.owl.math.Degree;
import ch.ethz.idsc.owl.math.StateSpaceModel;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.region.HyperplaneRegion;
import ch.ethz.idsc.owl.math.region.RegionUnion;
import ch.ethz.idsc.owl.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owl.math.state.StateIntegrator;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.demo.se2.CarStandardFlows;
import ch.ethz.idsc.owly.demo.se2.Se2CarIntegrator;
import ch.ethz.idsc.owly.demo.se2.Se2StateSpaceModel;
import ch.ethz.idsc.owly.demo.se2.glc.Se2Parameters;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

/** (x,y,theta) */
class Se2IterateGlcAnyStreetDemo {
  public static void main(String[] args) throws Exception {
    RationalScalar resolution = (RationalScalar) RealScalar.of(12);
    Scalar timeScale = RealScalar.of(10);
    Scalar depthScale = RealScalar.of(5);
    Tensor partitionScale = Tensors.vector(3, 3, 50 / Math.PI);
    Scalar dtMax = RationalScalar.of(1, 6);
    int maxIter = 2000;
    Tensor radiusVector = Tensors.of(DoubleScalar.of(0.2), DoubleScalar.of(0.2), Degree.of(15));
    StateSpaceModel stateSpaceModel = Se2StateSpaceModel.INSTANCE;
    // --
    Parameters parameters = new Se2Parameters( //
        resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, stateSpaceModel.getLipschitz());
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        Se2CarIntegrator.INSTANCE, parameters.getdtMax(), parameters.getTrajectorySize());
    // ---
    System.out.println("1/Domainsize=" + parameters.getEta());
    parameters.printResolution();
    CarStandardFlows carConfig = new CarStandardFlows(RealScalar.ONE, Degree.of(45));
    Collection<Flow> controls = carConfig.getFlows(6);
    Se2NoHeuristicGoalManager se2GoalManager = new Se2NoHeuristicGoalManager(Tensors.vector(-7, 0, 0), radiusVector);
    TrajectoryRegionQuery obstacleQuery = SimpleTrajectoryRegionQuery.timeInvariant( //
        RegionUnion.wrap(Arrays.asList( //
            new HyperplaneRegion(Tensors.vector(0, -1, 0), RealScalar.of(1)), //
            new HyperplaneRegion(Tensors.vector(0, +1, 0), RealScalar.of(1)) //
        )));
    // ---
    long tic = System.nanoTime();
    OptimalAnyTrajectoryPlanner trajectoryPlanner = new OptimalAnyTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, obstacleQuery, se2GoalManager.getGoalInterface());
    // ---
    trajectoryPlanner.insertRoot(new StateTime(Tensors.vector(-10, 0, 0), RealScalar.ZERO));
    int iters = GlcExpand.maxDepth(trajectoryPlanner, parameters.getDepthLimit());
    System.out.println("After " + iters + " iterations");
    long toc = System.nanoTime();
    System.out.println((toc - tic) * 1e-9 + " Seconds needed to plan");
    OwlyFrame owlyFrame = OwlyGui.start();
    owlyFrame.setGlc(trajectoryPlanner);
    // ---
    // --
    for (int iter = 0; iter < 300; iter++) {
      Thread.sleep(500);
      tic = System.nanoTime();
      Se2NoHeuristicGoalManager se2GoalManager2 = new Se2NoHeuristicGoalManager(Tensors.vector(-7 + iter, 0, 0), radiusVector);
      List<StateTime> trajectory = null;
      {
        Optional<GlcNode> optional = trajectoryPlanner.getBestOrElsePeek();
        if (optional.isPresent()) {
          trajectory = GlcNodes.getPathFromRootTo(optional.get());
        } else {
          throw new RuntimeException();
        }
      }
      StateTime newRootState = trajectory.get(1);
      // ---
      int increment = trajectoryPlanner.switchRootToState(newRootState.state());
      parameters.increaseDepthLimit(increment);
      System.out.println("New depthLimit is: " + parameters.getDepthLimit() + " after increment of " + increment);
      trajectoryPlanner.changeToGoal(se2GoalManager2.getGoalInterface());
      int iters2 = GlcExpand.maxDepth(trajectoryPlanner, parameters.getDepthLimit());
      StateTimeTrajectories.print(trajectory);
      // ---
      toc = System.nanoTime();
      System.out.println((toc - tic) * 1e-9 + " Seconds needed to replan");
      System.out.println("After root switch needed " + iters2 + " iterations");
      System.out.println("*****Finished*****");
      owlyFrame.setGlc(trajectoryPlanner);
      // owlyFrame.configCoordinateOffset(432, 273);
    }
  }
}
