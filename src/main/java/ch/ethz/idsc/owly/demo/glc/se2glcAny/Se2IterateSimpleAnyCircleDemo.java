// code by jl
package ch.ethz.idsc.owly.demo.glc.se2glcAny;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;

import ch.ethz.idsc.owly.demo.glc.se2.Se2Controls;
import ch.ethz.idsc.owly.demo.glc.se2.Se2GoalManager;
import ch.ethz.idsc.owly.demo.glc.se2.Se2StateSpaceModel;
import ch.ethz.idsc.owly.demo.glc.se2.Se2Utils;
import ch.ethz.idsc.owly.demo.glc.se2glc.Se2Parameters;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.SimpleAnyTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.wrap.Parameters;
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
class Se2IterateSimpleAnyCircleDemo {
  public static void main(String[] args) throws Exception {
    RationalScalar resolution = (RationalScalar) RealScalar.of(6);
    Scalar timeScale = RealScalar.of(10);
    Scalar depthScale = RealScalar.of(5);
    Tensor partitionScale = Tensors.vector(3, 3, 50 / Math.PI);
    Scalar dtMax = RationalScalar.of(1, 6);
    int maxIter = 2000;
    StateSpaceModel stateSpaceModel = new Se2StateSpaceModel();
    // --
    Parameters parameters = new Se2Parameters( //
        resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, stateSpaceModel.getLipschitz());
    // TODO possible without creation of StateSpaceModel?
    StateIntegrator stateIntegrator = FixedStateIntegrator.createDefault(parameters.getdtMax(), //
        parameters.getTrajectorySize());
    // ---
    System.out.println("1/Domainsize=" + parameters.getEta());
    parameters.printResolution();
    Collection<Flow> controls = Se2Controls.createControls(Se2Utils.DEGREE(45), 6);
    Se2GoalManager se2GoalManager = new Se2GoalManager( //
        Tensors.vector(3, 0), RealScalar.of(1.5 * Math.PI), // east
        DoubleScalar.of(.1), Se2Utils.DEGREE(10));
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            RegionUnion.of( //
                new EllipsoidRegion(Tensors.vector(0, 0, 0), Tensors.vector(1, 1, 10)), //
                new InvertedRegion(new EllipsoidRegion(Tensors.vector(0, 0, 0), Tensors.vector(5, 5, 10))), //
                new HyperplaneRegion(Tensors.vector(0, -1, 0), RealScalar.of(4)), //
                new HyperplaneRegion(Tensors.vector(0, +1, 0), RealScalar.of(4)) //
            )));
    // ---
    long tic = System.nanoTime();
    SimpleAnyTrajectoryPlanner trajectoryPlanner = new SimpleAnyTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, se2GoalManager, se2GoalManager.goalQuery(), obstacleQuery);
    // ---
    trajectoryPlanner.insertRoot(Tensors.vector(0, 3, 0));
    int iters = Expand.maxDepth(trajectoryPlanner, parameters.getDepthLimit());
    System.out.println("After " + iters + " iterations");
    List<StateTime> trajectory = trajectoryPlanner.getPathFromRootToGoal();
    long toc = System.nanoTime();
    System.out.println((toc - tic) * 1e-9 + " Seconds needed to plan");
    Trajectories.print(trajectory);
    OwlyFrame owlyFrame = Gui.start();
    owlyFrame.setGlc(trajectoryPlanner);
    // ---
    List<Tensor> goalListPosition = new ArrayList<>();
    goalListPosition.add(Tensors.vector(0, -3));// south
    goalListPosition.add(Tensors.vector(-3, 0));// West
    goalListPosition.add(Tensors.vector(0, 3)); // north
    goalListPosition.add(Tensors.vector(3, 0));// east
    List<RealScalar> goalListAngle = new ArrayList<>();
    goalListAngle.add(RealScalar.of(Math.PI));// South
    goalListAngle.add(RealScalar.of(0.5 * Math.PI));// west
    goalListAngle.add(RealScalar.of(0)); // north
    goalListAngle.add(RealScalar.of(-0.5 * Math.PI));// east
    // --
    Iterator<StateTime> trajectoryIterator = trajectory.iterator();
    trajectoryIterator.next();
    for (int iter = 0; iter < 100; iter++) {
      // while (trajectoryIterator.hasNext()) {
      Thread.sleep(500);
      tic = System.nanoTime();
      int index = iter % 4;
      System.out.println("index" + index);
      Se2GoalManager se2GoalManager2 = new Se2GoalManager( //
          goalListPosition.get(index), goalListAngle.get(index), //
          DoubleScalar.of(0.1), Se2Utils.DEGREE(10));
      StateTime newRootState = trajectory.get(1);
      // System.out.println("trajectory at "+ (iter+1));
      // ---
      trajectoryPlanner.switchRootToState(newRootState.x());
      trajectoryPlanner.setGoalQuery(se2GoalManager2, se2GoalManager2.goalQuery());
      int iters2 = Expand.maxDepth(trajectoryPlanner, parameters.getDepthLimit());
      trajectory = trajectoryPlanner.getPathFromRootToGoal();
      Trajectories.print(trajectory);
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
