// code by jl
package ch.ethz.idsc.owly.demo.rn.any;

import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.demo.rn.R2Controls;
import ch.ethz.idsc.owly.demo.rn.R2NoiseRegion;
import ch.ethz.idsc.owly.demo.rn.R2Parameters;
import ch.ethz.idsc.owly.demo.rn.RnSimpleCircleGoalManager;
import ch.ethz.idsc.owly.glc.adapter.Parameters;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.AnyPlannerInterface;
import ch.ethz.idsc.owly.glc.core.GlcExpand;
import ch.ethz.idsc.owly.glc.core.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.gui.OwlyFrame;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.region.InvertedRegion;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

enum R2GlcAnyObstacleChangeDemo {
  ;
  public static void main(String[] args) throws Exception {
    RationalScalar resolution = (RationalScalar) RealScalar.of(10);
    Scalar timeScale = RealScalar.of(2);
    Scalar depthScale = RealScalar.of(100);
    Tensor partitionScale = Tensors.vector(30, 30);
    Scalar dtMax = RationalScalar.of(1, 6);
    int maxIter = 2000;
    Scalar lipschitz = RealScalar.ONE;
    Tensor goal = Tensors.vector(0, -7);
    Scalar goalRadius = DoubleScalar.of(.25);
    System.out.println("Goal is: " + goal);
    Parameters parameters = new R2Parameters( //
        resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, lipschitz);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create(EulerIntegrator.INSTANCE, parameters.getdtMax(), //
        parameters.getTrajectorySize());
    Collection<Flow> controls = R2Controls.createRadial(parameters.getResolutionInt()); // max (grad(F)) ==1
    RnSimpleCircleGoalManager rnGoal = new RnSimpleCircleGoalManager(goal, goalRadius);
    // performance depends on heuristic: zeroHeuristic vs rnGoal
    // Heuristic heuristic = new ZeroHeuristic(); // rnGoal
    TrajectoryRegionQuery obstacleQuery1 = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            RegionUnion.of( //
                new EllipsoidRegion(Tensors.vector(0, 0), Tensors.vector(3, 3))//
                , new InvertedRegion(new EllipsoidRegion(Tensors.vector(0, 0), Tensors.vector(8, 8)))
                // , RnPointclouds.createRandomRegion(30, Tensors.vector(12, 12), Tensors.vector(0, 0), RealScalar.of(0.6)) //
                , new R2NoiseRegion(.2)//
            )));
    // --
    AnyPlannerInterface trajectoryPlanner = new OptimalAnyTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, obstacleQuery1, rnGoal);
    trajectoryPlanner.switchRootToState(Tensors.vector(0, 6));
    GlcExpand.maxDepth(trajectoryPlanner, parameters.getDepthLimit());
    List<StateTime> trajectory = trajectoryPlanner.trajectoryToBest();
    StateTimeTrajectories.print(trajectory);
    OwlyFrame owlyFrame = Gui.start();
    owlyFrame.configCoordinateOffset(400, 400);
    owlyFrame.jFrame.setBounds(0, 0, 800, 800);
    owlyFrame.setGlc((TrajectoryPlanner) trajectoryPlanner);
    TrajectoryRegionQuery obstacleQuery2 = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            RegionUnion.of( //
                // new EllipsoidRegion(Tensors.vector(0, 0), Tensors.vector(3, 3)),//
                new InvertedRegion(new EllipsoidRegion(Tensors.vector(0, 0), Tensors.vector(8, 8))),
                // RnPointclouds.createRandomRegion(30, Tensors.vector(12, 12), Tensors.vector(0, 0), RealScalar.of(0.6)), //
                new R2NoiseRegion(.2)//
            )));
    trajectoryPlanner.obstacleUpdate(obstacleQuery2);
    owlyFrame.setGlc((TrajectoryPlanner) trajectoryPlanner);
    GlcExpand.constTime(trajectoryPlanner, RealScalar.of(1.5), parameters.getDepthLimit());
    owlyFrame.setGlc((TrajectoryPlanner) trajectoryPlanner);
  }
}
