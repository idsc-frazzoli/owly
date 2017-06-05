// code by jl
package ch.ethz.idsc.owly.demo.glc.se2glcAny;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.demo.glc.se2.Se2Controls;
import ch.ethz.idsc.owly.demo.glc.se2.Se2GoalManager;
import ch.ethz.idsc.owly.demo.glc.se2.Se2StateSpaceModel;
import ch.ethz.idsc.owly.demo.glc.se2.Se2Utils;
import ch.ethz.idsc.owly.demo.glc.se2glc.Se2Parameters;
import ch.ethz.idsc.owly.glc.adapter.Parameters;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.AnyTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.gui.OwlyFrame;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.region.HyperplaneRegion;
import ch.ethz.idsc.owly.math.region.InvertedRegion;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.Trajectories;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

/** (x,y,theta) */
class Se2IterateAnyCircleDemo {
  public static void main(String[] args) throws Exception {
    RationalScalar resolution = (RationalScalar) RealScalar.of(10);
    Scalar timeScale = RealScalar.of(4);
    Scalar depthScale = RealScalar.of(10);
    Tensor partitionScale = Tensors.vector(3, 3, 50 / Math.PI);
    Scalar dtMax = RationalScalar.of(1, 12);
    int maxIter = 2000;
    StateSpaceModel stateSpaceModel = new Se2StateSpaceModel();
    // --
    Parameters parameters = new Se2Parameters( //
        resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, stateSpaceModel.getLipschitz());
    // TODO JONAS what do you mean: possible without creation of StateSpaceModel?
    StateIntegrator stateIntegrator = FixedStateIntegrator.createDefault(parameters.getdtMax(), //
        parameters.getTrajectorySize());
    // ---
    System.out.println("1/Domainsize=" + parameters.getEta());
    parameters.printResolution();
    Collection<Flow> controls = Se2Controls.createControls(Se2Utils.DEGREE(45), parameters.getResolution());
    Se2GoalManager se2GoalManager = new Se2GoalManager( //
        Tensors.vector(3, 0), RealScalar.of(1.5 * Math.PI), // east
        DoubleScalar.of(.1), Se2Utils.DEGREE(10));
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            RegionUnion.of( //
                new EllipsoidRegion(Tensors.vector(0, 0, 0), Tensors.vector(1, 1, Double.POSITIVE_INFINITY)), //
                new InvertedRegion(new EllipsoidRegion(Tensors.vector(0, 0, 0), Tensors.vector(5, 5, Double.POSITIVE_INFINITY))), //
                new HyperplaneRegion(Tensors.vector(0, -1, 0), RealScalar.of(4)), //
                new HyperplaneRegion(Tensors.vector(0, +1, 0), RealScalar.of(4)) //
            )));
    // ---
    Scalar tic = RealScalar.of(System.nanoTime());
    AnyTrajectoryPlanner trajectoryPlanner = new AnyTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, se2GoalManager, se2GoalManager.goalQuery(), obstacleQuery);
    // ---
    trajectoryPlanner.insertRoot(Tensors.vector(0, 3, 0));
    int iters = Expand.maxDepth(trajectoryPlanner, parameters.getDepthLimit());
    System.out.println("After " + iters + " iterations");
    List<StateTime> trajectory = trajectoryPlanner.getPathFromRootToGoal();
    Scalar toc = RealScalar.of(System.nanoTime());
    System.out.println(toc.subtract(tic).multiply(RealScalar.of(1e-9)) + " Seconds needed to plan");
    Trajectories.print(trajectory);
    OwlyFrame owlyFrame = Gui.start();
    owlyFrame.setGlc(trajectoryPlanner);
    // ---
    List<Tensor> goalListPosition = new ArrayList<>();
    goalListPosition.add(Tensors.vector(0, -3));// South
    goalListPosition.add(Tensors.vector(-3, 0));// West
    goalListPosition.add(Tensors.vector(0, 3)); // North
    goalListPosition.add(Tensors.vector(3, 0)); // East
    List<RealScalar> goalListAngle = new ArrayList<>();
    goalListAngle.add(RealScalar.of(Math.PI)); // South
    goalListAngle.add(RealScalar.of(0.5 * Math.PI)); // West
    goalListAngle.add(RealScalar.of(0)); // North
    goalListAngle.add(RealScalar.of(-0.5 * Math.PI)); // East
    // --
    int iter = 0;
    Scalar timeSum = RealScalar.of(0);
    while (owlyFrame.jFrame.isVisible()) {
      Scalar delay = RealScalar.of(1000);
      Thread.sleep(500);
      tic = RealScalar.of(System.nanoTime());
      int index = iter % 4;
      Se2GoalManager se2GoalManager2 = new Se2GoalManager( //
          goalListPosition.get(index), goalListAngle.get(index), //
          DoubleScalar.of(0.1), Se2Utils.DEGREE(10));
      // --
      StateTime newRootState = trajectory.get(8);
      // GlcNode newRootNode = trajectoryPlanner.getNodesfromRootToGoal().get(1);
      // trajectoryPlanner.switchRootToNode(newRootNode);
      int increment = trajectoryPlanner.switchRootToState(newRootState.x());
      parameters.increaseDepthLimit(increment);
      Thread.sleep(delay.number().intValue() / 2);
      // --
      trajectoryPlanner.setGoalQuery(se2GoalManager2, se2GoalManager2.goalQuery());
      Thread.sleep(delay.number().intValue() / 2);
      // --
      int expandIter = Expand.maxDepth(trajectoryPlanner, parameters.getDepthLimit());
      // ---
      toc = RealScalar.of(System.nanoTime());
      trajectory = trajectoryPlanner.getPathFromRootToGoal();
      Trajectories.print(trajectory);
      timeSum = toc.subtract(tic).multiply(RealScalar.of(1e-9)).add(timeSum);
      System.out.println((iter + 1) + ". iteration took: " + toc.subtract(tic).multiply(RealScalar.of(1e-9)) + "s");
      System.out.println("Average: " + timeSum.divide(RealScalar.of(iter + 1)));
      System.out.println("After root switch needed " + expandIter + " iterations");
      System.out.println("*****Finished*****");
      System.out.println("");
      owlyFrame.setGlc(trajectoryPlanner);
      // owlyFrame.configCoordinateOffset(432, 273);
      iter++;
    }
  }
}
