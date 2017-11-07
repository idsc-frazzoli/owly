// code by jl
package ch.ethz.idsc.owly.demo.deltaxt;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.data.Lists;
import ch.ethz.idsc.owly.demo.delta.DeltaControls;
import ch.ethz.idsc.owly.demo.delta.DeltaParameters;
import ch.ethz.idsc.owly.demo.delta.ImageGradient;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.demo.util.UserHome;
import ch.ethz.idsc.owly.glc.adapter.Parameters;
import ch.ethz.idsc.owly.glc.adapter.RxtTimeInvariantRegion;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.Trajectories;
import ch.ethz.idsc.owly.glc.core.DebugUtils;
import ch.ethz.idsc.owly.glc.core.GlcExpand;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectorySample;
import ch.ethz.idsc.owly.gui.ani.OwlyFrame;
import ch.ethz.idsc.owly.gui.ani.OwlyGui;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.RungeKutta45Integrator;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.io.AnimationWriter;
import ch.ethz.idsc.tensor.io.ResourceData;

public class DeltaxtGlcDemo implements DemoInterface {
  ;
  @Override
  public void start() {
    // SETUP
    RationalScalar resolution = (RationalScalar) RationalScalar.of(12, 1);
    Tensor partitionScale = Tensors.vector(2e12, 2e12, 2e12);
    Scalar timeScale = RealScalar.of(5);
    Scalar depthScale = RealScalar.of(10);
    Scalar dtMax = RationalScalar.of(1, 6);
    int maxIter = 200000;
    Tensor range = Tensors.vector(9, 6.5);
    AnimationWriter gsw = null;
    try {
      gsw = AnimationWriter.of(UserHome.Pictures("delta_s.gif"), 250);
    } catch (Exception exception) {
      exception.printStackTrace();
    }
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
    List<Region<Tensor>> goalRegions = new ArrayList<>();
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
      next = Lists.getLast(connector);
      goalRegions.add(new EllipsoidRegion(next.state(), radius));
      dinghyTrajectory.add(new TrajectorySample(next, flow));
    }
    Trajectories.print(dinghyTrajectory);
    // GOALCREATION
    DeltaxtDinghyGoalManager deltaGoalManager2 = new DeltaxtDinghyGoalManager(goalRegions, stateSpaceModel);
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, obstacleQuery, deltaGoalManager2);
    trajectoryPlanner.insertRoot(new StateTime(Tensors.vector(8.8, 0.5, 0), RealScalar.ZERO));
    // RUN
    OwlyFrame owlyFrame = OwlyGui.start();
    owlyFrame.configCoordinateOffset(33, 416);
    owlyFrame.jFrame.setBounds(100, 100, 620, 525);
    owlyFrame.addRegionRender(imageRegion);
    owlyFrame.addTrajectory(dinghyTrajectory, new Color(224, 168, 0, 224));
    while (!trajectoryPlanner.getBest().isPresent() && owlyFrame.jFrame.isVisible()) {
      GlcExpand.maxSteps(trajectoryPlanner, 30, parameters.getDepthLimit());
      owlyFrame.setGlc(trajectoryPlanner);
      try {
        gsw.append(owlyFrame.offscreen());
      } catch (Exception e1) {
        e1.printStackTrace();
      }
      try {
        Thread.sleep(1);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
      DebugUtils.heuristicConsistencyCheck(trajectoryPlanner);
      if (trajectoryPlanner.getQueue().isEmpty())
        break;
      if (trajectoryPlanner.getBest().isPresent())
        owlyFrame.addTrajectory(dinghyTrajectory, new Color(224, 168, 0, 224));
    }
    int repeatLast = 6;
    while (0 < repeatLast--)
      try {
        gsw.append(owlyFrame.offscreen());
      } catch (Exception e) {
        e.printStackTrace();
      }
    try {
      gsw.close();
    } catch (Exception e) {
      e.printStackTrace();
    }
    System.out.println("created gif");
  }

  public static void main(String[] args) {
    new DeltaxtGlcDemo().start();
  }
}
