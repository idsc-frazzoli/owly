// code by jph
package ch.ethz.idsc.owly.demo.delta.glc;

import java.util.Collection;

import ch.ethz.idsc.owly.demo.delta.DeltaControls;
import ch.ethz.idsc.owly.demo.delta.DeltaMinTimeGoalManager;
import ch.ethz.idsc.owly.demo.delta.DeltaStateSpaceModel;
import ch.ethz.idsc.owly.demo.delta.ImageGradient;
import ch.ethz.idsc.owly.demo.util.UserHome;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.ani.OwlyFrame;
import ch.ethz.idsc.owly.gui.ani.OwlyGui;
import ch.ethz.idsc.owly.gui.region.RegionRenders;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.RungeKutta45Integrator;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.region.SphericalRegion;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.io.AnimationWriter;
import ch.ethz.idsc.tensor.io.ResourceData;

/** simple animation of small boat driving upstream, or downstream in a river delta
 * 
 * records to animated gif */
enum DeltaExpandDemo {
  ;
  public static void main(String[] args) throws Exception {
    Scalar amp = RealScalar.of(.5); // -.25 .5
    // ---
    Tensor eta = Tensors.vector(8, 8);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        RungeKutta45Integrator.INSTANCE, RationalScalar.of(1, 10), 4);
    Tensor range = Tensors.vector(9, 6.5);
    ImageGradient ipr = new ImageGradient(ResourceData.of("/io/delta_uxy.png"), range, amp);
    Scalar maxInput = RealScalar.ONE;
    StateSpaceModel stateSpaceModel = new DeltaStateSpaceModel(ipr, maxInput);
    Collection<Flow> controls = DeltaControls.createControls(stateSpaceModel, maxInput, 25);
    Tensor obstacleImage = ResourceData.of("/io/delta_free.png"); //
    Region<Tensor> region = new ImageRegion(obstacleImage, range, true);
    TrajectoryRegionQuery obstacleQuery = SimpleTrajectoryRegionQuery.timeInvariant(region);
    Scalar maxMove = stateSpaceModel.getLipschitz();
    SphericalRegion sphericalRegion = new SphericalRegion(Tensors.vector(2.1, 0.3), RealScalar.of(.3));
    GoalInterface goalInterface = new DeltaMinTimeGoalManager(sphericalRegion, maxMove);
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        eta, stateIntegrator, controls, obstacleQuery, goalInterface);
    trajectoryPlanner.insertRoot(new StateTime(Tensors.vector(8.8, 0.5), RealScalar.ZERO));
    // ---
    OwlyFrame owlyFrame = OwlyGui.start();
    owlyFrame.addBackground(RegionRenders.create(region));
    owlyFrame.addBackground(RegionRenders.create(sphericalRegion));
    owlyFrame.addBackground(DeltaHelper.vectorFieldRender(stateSpaceModel, range, region, RealScalar.of(0.05)));
    owlyFrame.configCoordinateOffset(33, 416);
    owlyFrame.jFrame.setBounds(100, 100, 620, 475);
    AnimationWriter gsw = AnimationWriter.of(UserHome.Pictures("delta_s.gif"), 250);
    while (!trajectoryPlanner.getBest().isPresent() && owlyFrame.jFrame.isVisible()) {
      Expand.maxSteps(trajectoryPlanner, 40);
      owlyFrame.setGlc(trajectoryPlanner);
      gsw.append(owlyFrame.offscreen());
      Thread.sleep(1);
    }
    int repeatLast = 6;
    while (0 < repeatLast--)
      gsw.append(owlyFrame.offscreen());
    gsw.close();
    System.out.println("created gif");
  }
}
