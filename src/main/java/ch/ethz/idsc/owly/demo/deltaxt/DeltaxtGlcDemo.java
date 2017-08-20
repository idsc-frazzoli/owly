// code by jl
package ch.ethz.idsc.owly.demo.deltaxt;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.demo.delta.DeltaControls;
import ch.ethz.idsc.owly.demo.delta.DeltaParameters;
import ch.ethz.idsc.owly.demo.delta.ImageGradient;
import ch.ethz.idsc.owly.glc.adapter.Parameters;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.adapter.Trajectories;
import ch.ethz.idsc.owly.glc.core.DebugUtils;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectorySample;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.gui.OwlyFrame;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.RungeKutta45Integrator;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.RxtTimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.io.ResourceData;

enum DeltaxtGlcDemo {
  ;
  public static void main(String[] args) throws Exception {
    // SETUP
    RationalScalar resolution = (RationalScalar) RationalScalar.of(12, 1);
    Tensor partitionScale = Tensors.vector(4e9, 4e9, 4e9);
    Scalar timeScale = RealScalar.of(5);
    Scalar depthScale = RealScalar.of(10);
    Scalar dtMax = RationalScalar.of(1, 6);
    int maxIter = 200000;
    Tensor range = Tensors.vector(9, 6.5);
    ImageGradient ipr = new ImageGradient(ResourceData.of("/io/delta_uxy.png"), range, RealScalar.of(-0.1)); // -.25 .5
    Scalar maxInput = RealScalar.ONE;
    DeltaxtStateSpaceModel stateSpaceModel = new DeltaxtStateSpaceModel(ipr, maxInput);
    Collection<Flow> controls = DeltaControls.createControls( //
        stateSpaceModel, maxInput, resolution.number().intValue());
    Parameters parameters = new DeltaParameters(resolution, timeScale, depthScale, //
        partitionScale, dtMax, maxIter, stateSpaceModel.getLipschitz());
    System.out.println("1/DomainSize: " + parameters.getEta());
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        RungeKutta45Integrator.INSTANCE, parameters.getdtMax(), parameters.getTrajectorySize());
    Tensor obstacleImage = ResourceData.of("/io/delta_free.png"); //
    ImageRegion imageRegion = new ImageRegion(obstacleImage, range, true);
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new RxtTimeInvariantRegion( //
            imageRegion));
    // Creating Dinghy trajectory
    List<Region> goalRegions = new ArrayList<>();
    List<TrajectorySample> dinghyTrajectory = new ArrayList<>();
    Tensor radius = Tensors.vector(0.3, 0.3, 2);
    // Start of dinghy
    StateTime next = new StateTime(Tensors.vector(1.7, 2.100, 0), RealScalar.ZERO);
    // StateTime next = new StateTime(Tensors.vector(1.3, 2.100, 0), RealScalar.ZERO);
    goalRegions.add(new EllipsoidRegion(next.state(), radius));
    dinghyTrajectory.add(new TrajectorySample(next, null));
    Scalar dinghyExpandTime = RealScalar.of(25); // [s]
    for (int i = 0; Scalars.lessThan(RealScalar.of(i), dinghyExpandTime.divide(parameters.getExpandTime())); i++) {
      Flow flow = StateSpaceModels.createFlow(stateSpaceModel, Tensors.vector(0, 0));
      List<StateTime> connector = stateIntegrator.trajectory(next, flow);
      next = StateTimeTrajectories.getLast(connector);
      goalRegions.add(new EllipsoidRegion(next.state(), radius));
      dinghyTrajectory.add(new TrajectorySample(next, flow));
    }
    Trajectories.print(dinghyTrajectory);
    // GOALCREATION
    DeltaxtDinghyGoalManager deltaGoalManager2 = new DeltaxtDinghyGoalManager(goalRegions, stateSpaceModel);
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, obstacleQuery, deltaGoalManager2);
    trajectoryPlanner.insertRoot(Tensors.vector(8.8, 0.5, 0));
    // RUN
    OwlyFrame owlyFrame = Gui.start();
    owlyFrame.configCoordinateOffset(33, 416);
    owlyFrame.jFrame.setBounds(100, 100, 620, 475);
    owlyFrame.addBackground(imageRegion);
    owlyFrame.addTrajectory(dinghyTrajectory, new Color(224, 168, 0, 224)); // add goalTrajectory Goalcolor
    while (!trajectoryPlanner.getBest().isPresent() && owlyFrame.jFrame.isVisible()) {
      Expand.maxSteps(trajectoryPlanner, 30, parameters.getDepthLimit());
      owlyFrame.setGlc(trajectoryPlanner);
      Thread.sleep(1);
      DebugUtils.heuristicConsistencyCheck(trajectoryPlanner);
      if (trajectoryPlanner.getQueue().isEmpty())
        break;
    }
  }
}
