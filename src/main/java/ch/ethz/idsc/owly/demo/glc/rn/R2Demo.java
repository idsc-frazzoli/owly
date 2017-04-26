// code by jph
package ch.ethz.idsc.owly.demo.glc.rn;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.EllipsoidRegion;
import ch.ethz.idsc.owly.glc.adapter.InvertedRegion;
import ch.ethz.idsc.owly.glc.adapter.MinTimeCost;
import ch.ethz.idsc.owly.glc.adapter.RnPointcloudRegion;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.TimeInvariantRegion;
import ch.ethz.idsc.owly.glc.core.Controls;
import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.DefaultTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.Heuristic;
import ch.ethz.idsc.owly.glc.core.StateTime;
import ch.ethz.idsc.owly.glc.core.Trajectory;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.gui.GlcFrame;
import ch.ethz.idsc.owly.math.RegionUnion;
import ch.ethz.idsc.owly.math.integrator.EulerIntegrator;
import ch.ethz.idsc.owly.math.integrator.Integrator;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public class R2Demo {
  public static R2Demo createSphere() {
    R2Demo rnDemo = new R2Demo();
    rnDemo.controlSize = 40;
    rnDemo.root = Tensors.vector(0, 0);
    rnDemo.goal = new RnGoal(Tensors.vector(5, 0), DoubleScalar.of(.2));
    rnDemo.obstacleQuery = // new EmptyRegionQuery();
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            RegionUnion.of( //
                new EllipsoidRegion(Tensors.vector(4, 3), Tensors.vector(2, 2)), //
                new EllipsoidRegion(Tensors.vector(2.5, 0), Tensors.vector(2, 2)) //
            )));
    return rnDemo;
  }

  public static R2Demo createPoints() {
    R2Demo rnDemo = new R2Demo();
    rnDemo.controlSize = 40;
    rnDemo.root = Tensors.vector(0, 0);
    rnDemo.goal = new RnGoal(Tensors.vector(5, 0), DoubleScalar.of(.2));
    Tensor points = Tensors.matrix(new Number[][] { //
        { 2.5, 1 }, { 1.5, -1.5 }, { 0, 2 }, { 3.5, -0.5 } //
    });
    rnDemo.obstacleQuery = // new EmptyRegionQuery();
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            RnPointcloudRegion.create(points, RealScalar.ONE)));
    return rnDemo;
  }

  public static R2Demo createPointsInside() {
    R2Demo rnDemo = new R2Demo();
    rnDemo.controlSize = 40;
    rnDemo.root = Tensors.vector(0, 0);
    rnDemo.goal = new RnGoal(Tensors.vector(5, 0), DoubleScalar.of(.2));
    Tensor points = Tensors.matrix(new Number[][] { //
        { 0, 0 }, { 0, -1 }, { 0, -2 }, //
        { 1, -2 }, { 2, -2 }, { 3, -2 }, { 4, -2 }, //
        { 5, -2 }, { 5, -1 }, { 5, 0 } //
    });
    rnDemo.obstacleQuery = // new EmptyRegionQuery();
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            new InvertedRegion(RnPointcloudRegion.create(points, RealScalar.of(0.6)))));
    return rnDemo;
  }

  public static R2Demo createBubbles() {
    R2Demo rnDemo = new R2Demo();
    rnDemo.controlSize = 15;
    rnDemo.root = Tensors.vector(-2, -2);
    rnDemo.goal = new RnGoal(Tensors.vector(2, 2), DoubleScalar.of(.25));
    rnDemo.obstacleQuery = new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(new R2Bubbles()));
    return rnDemo;
  }

  int controlSize;
  Tensor root;
  RnGoal goal;
  TrajectoryRegionQuery obstacleQuery;

  private R2Demo() {
  }

  public static void main(String[] args) {
    R2Demo rnDemo = createBubbles();
    rnDemo = createSphere();
    Integrator integrator = new EulerIntegrator();
    final Scalar timeStep = RealScalar.of(.20);
    Tensor partitionScale = Tensors.vector(7, 7);
    Controls controls = new R2Controls(rnDemo.controlSize, RealScalar.ONE);
    int trajectorySize = 5;
    CostFunction costFunction = new MinTimeCost();
    Heuristic heuristic = rnDemo.goal;
    TrajectoryRegionQuery goalQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(rnDemo.goal));
    TrajectoryRegionQuery obstacleQuery = rnDemo.obstacleQuery;
    // ---
    TrajectoryPlanner trajectoryPlanner = new DefaultTrajectoryPlanner( //
        integrator, timeStep, partitionScale, controls, trajectorySize, costFunction, heuristic, goalQuery, obstacleQuery);
    trajectoryPlanner.insertRoot(rnDemo.root);
    trajectoryPlanner.plan(45);
    List<StateTime> trajectory = trajectoryPlanner.getPathFromRootToGoal();
    Trajectory.print(trajectory);
    GlcFrame glcFrame = new GlcFrame();
    glcFrame.glcComponent.setTrajectoryPlanner(trajectoryPlanner);
  }
}
