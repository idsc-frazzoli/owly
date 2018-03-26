// code by jl
package ch.ethz.idsc.owly.demo.delta.glc;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owl.data.Lists;
import ch.ethz.idsc.owl.glc.adapter.GlcExpand;
import ch.ethz.idsc.owl.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owl.glc.adapter.Trajectories;
import ch.ethz.idsc.owl.glc.core.DebugUtils;
import ch.ethz.idsc.owl.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owl.glc.par.Parameters;
import ch.ethz.idsc.owl.glc.std.StandardTrajectoryPlanner;
import ch.ethz.idsc.owl.gui.ani.OwlyFrame;
import ch.ethz.idsc.owl.gui.ani.OwlyGui;
import ch.ethz.idsc.owl.math.StateSpaceModels;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.flow.RungeKutta45Integrator;
import ch.ethz.idsc.owl.math.region.ImageRegion;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.owl.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owl.math.state.StateIntegrator;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owl.math.state.TrajectorySample;
import ch.ethz.idsc.owly.demo.delta.DeltaFlows;
import ch.ethz.idsc.owly.demo.delta.DeltaParameters;
import ch.ethz.idsc.owly.demo.delta.DeltaStateSpaceModel;
import ch.ethz.idsc.owly.demo.delta.ImageGradient;
import ch.ethz.idsc.owly.demo.rn.R2xTEllipsoidStateTimeRegion;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.demo.util.RegionRenders;
import ch.ethz.idsc.owly.demo.util.TrajectoryTranslationFamily;
import ch.ethz.idsc.owly.demo.util.UserHome;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.io.AnimationWriter;
import ch.ethz.idsc.tensor.io.ResourceData;

public class DeltaxTGlcDemo implements DemoInterface {
  @Override
  public void start() {
    // SETUP
    RationalScalar resolution = (RationalScalar) RationalScalar.of(9, 1);
    Tensor partitionScale = Tensors.vector(150, 150, 300);
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
    ImageGradient ipr = ImageGradient.linear(ResourceData.of("/io/delta_uxy.png"), range, RealScalar.of(-0.1));
    Scalar maxInput = RealScalar.ONE;
    DeltaStateSpaceModel stateSpaceModel = new DeltaStateSpaceModel(ipr);
    Parameters parameters = new DeltaParameters(resolution, timeScale, depthScale, //
        partitionScale, dtMax, maxIter, stateSpaceModel.getLipschitz());
    DeltaFlows deltaFlows = new DeltaFlows(stateSpaceModel, maxInput);
    Collection<Flow> controls = deltaFlows.getFlows(parameters.getResolutionInt() - 1);
    controls.add(deltaFlows.stayPut());
    System.out.println("1/DomainSize: " + parameters.getEta());
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        RungeKutta45Integrator.INSTANCE, parameters.getdtMax(), parameters.getTrajectorySize());
    Tensor obstacleImage = ResourceData.of("/io/delta_free.png"); //
    ImageRegion imageRegion = new ImageRegion(obstacleImage, range, true);
    TrajectoryRegionQuery obstacleQuery = SimpleTrajectoryRegionQuery.timeInvariant(imageRegion);
    // Creating DINGHY TRAJECTORY
    List<TrajectorySample> dinghyTrajectory = new ArrayList<>();
    Tensor radius = Tensors.vector(0.3, 0.3);
    // Start of dinghy
    StateTime next = new StateTime(Tensors.vector(1.7, 2.100), RealScalar.ZERO);
    StateIntegrator dinghyStateIntegrator = FixedStateIntegrator.create( //
        RungeKutta45Integrator.INSTANCE, parameters.getdtMax(), RealScalar.of(25).divide(parameters.getdtMax()).number().intValue());
    dinghyTrajectory.add(new TrajectorySample(next, null));
    Scalar dinghyExpandTime = RealScalar.of(25); // [s]
    Flow flow = StateSpaceModels.createFlow(stateSpaceModel, Tensors.vector(0, 0));
    Region<StateTime> goalRegion = new R2xTEllipsoidStateTimeRegion(radius, //
        TrajectoryTranslationFamily.create(dinghyStateIntegrator, next, flow), //
        null);
    // all state times in trajectory members of goalRegion
    // for visualization purposes
    for (int i = 0; Scalars.lessThan(RealScalar.of(i), dinghyExpandTime.divide(parameters.getExpandTime())); i++) {
      next = Lists.getLast(stateIntegrator.trajectory(next, flow));
      dinghyTrajectory.add(new TrajectorySample(next, flow));
    }
    Trajectories.print(dinghyTrajectory);
    // GOALCREATION
    DeltaxTDinghyGoalManager deltaGoalManager = new DeltaxTDinghyGoalManager(goalRegion, stateSpaceModel, RealScalar.of(0.001));
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, obstacleQuery, deltaGoalManager);
    trajectoryPlanner.represent = StateTime::joined;
    trajectoryPlanner.insertRoot(new StateTime(Tensors.vector(8.8, 0.5), RealScalar.ZERO));
    // RUN
    OwlyFrame owlyFrame = OwlyGui.start();
    owlyFrame.configCoordinateOffset(33, 416);
    owlyFrame.jFrame.setBounds(100, 100, 620, 525);
    owlyFrame.addBackground(RegionRenders.create(imageRegion));
    owlyFrame.addTrajectory(dinghyTrajectory, new Color(224, 168, 0, 224));
    while (!trajectoryPlanner.getBest().isPresent() && owlyFrame.jFrame.isVisible()) {
      GlcExpand.maxSteps(trajectoryPlanner, 100, parameters.getDepthLimit());
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
    new DeltaxTGlcDemo().start();
  }
}
