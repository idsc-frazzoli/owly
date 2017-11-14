// code by jph
package ch.ethz.idsc.owly.demo.delta.glc;

import java.util.Arrays;

import ch.ethz.idsc.owly.demo.delta.ImageGradient;
import ch.ethz.idsc.owly.demo.rn.R2xTEllipsoidStateTimeRegion;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.demo.util.TrajectoryTranslationFamily;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.gui.ani.AbstractEntity;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.gui.region.RegionRenders;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.RungeKutta45Integrator;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.se2.RigidFamily;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.io.ResourceData;

public class DeltaxTAnimationDemo implements DemoInterface {
  @Override
  public void start() {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    // ---
    Scalar amp = RealScalar.of(-.05);
    Tensor range = Tensors.vector(12.6, 9.1).unmodifiable();
    ImageGradient imageGradient = ImageGradient.linear(ResourceData.of("/io/delta_uxy.png"), range, amp);
    StateSpaceModel stateSpaceModel = DeltaEntity.model(imageGradient);
    Tensor obstacleImage = ResourceData.of("/io/delta_free.png");
    ImageRegion imageRegion = new ImageRegion(obstacleImage, range, true);
    AbstractEntity abstractEntity = new DeltaxTEntity(imageGradient, Tensors.vector(10, 3.5));
    // ---
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        RungeKutta45Integrator.INSTANCE, RationalScalar.of(1, 10), 120 * 10);
    Flow flow = StateSpaceModels.createFlow(stateSpaceModel, DeltaEntity.FALLBACK_CONTROL);
    RigidFamily rigidFamily = TrajectoryTranslationFamily.create(stateIntegrator, new StateTime(Tensors.vector(2, 1.5), RealScalar.ZERO), flow);
    Region<StateTime> region1 = new R2xTEllipsoidStateTimeRegion(Tensors.vector(0.4, 0.4), //
        rigidFamily, () -> abstractEntity.getStateTimeNow().time());
    Region<StateTime> region2 = new R2xTEllipsoidStateTimeRegion(Tensors.vector(0.5, 0.5), //
        TrajectoryTranslationFamily.create(stateIntegrator, new StateTime(Tensors.vector(6, 6), RealScalar.ZERO), flow), //
        () -> abstractEntity.getStateTimeNow().time());
    // ---
    TrajectoryRegionQuery obstacleQuery = new SimpleTrajectoryRegionQuery( //
        RegionUnion.wrap(Arrays.asList(new TimeInvariantRegion(imageRegion), region1, region2)));
    owlyAnimationFrame.set(abstractEntity);
    owlyAnimationFrame.setObstacleQuery(obstacleQuery);
    owlyAnimationFrame.addBackground(RegionRenders.create(imageRegion));
    owlyAnimationFrame.addBackground((RenderInterface) region1);
    owlyAnimationFrame.addBackground((RenderInterface) region2);
    owlyAnimationFrame.addBackground(DeltaHelper.vectorFieldRender(stateSpaceModel, range, imageRegion, RealScalar.of(0.5)));
    owlyAnimationFrame.jFrame.setVisible(true);
    owlyAnimationFrame.configCoordinateOffset(50, 600);
  }

  public static void main(String[] args) throws Exception {
    new DeltaxTAnimationDemo().start();
  }
}
