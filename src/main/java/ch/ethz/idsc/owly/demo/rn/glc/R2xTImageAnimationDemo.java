// code by jph
package ch.ethz.idsc.owly.demo.rn.glc;

import java.util.Arrays;

import ch.ethz.idsc.owly.demo.rn.R2ImageRegions;
import ch.ethz.idsc.owly.demo.rn.R2xTImageStateTimeRegion;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.demo.util.RigidFamily;
import ch.ethz.idsc.owly.demo.util.So2Family;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.gui.ani.AbstractEntity;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;

public class R2xTImageAnimationDemo implements DemoInterface {
  @Override
  public void start() {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    AbstractEntity abstractEntity = new R2xTEntity(Tensors.vector(1.2, 2.2), RealScalar.of(1.5));
    owlyAnimationFrame.set(abstractEntity);
    // ---
    RigidFamily rigid1 = new So2Family(time -> time.multiply(RealScalar.of(.2)));
    // new TranslationFamily(s -> Tensors.of( //
    // Sin.of(s.multiply(RealScalar.of(.3))), //
    // Cos.of(s.multiply(RealScalar.of(.2))) //
    // ));
    // new Se2Family( //
    // scalar -> Tensors.of( //
    // Cos.FUNCTION.apply(scalar.multiply(RealScalar.of(.1))).multiply(RealScalar.of(2.0)), //
    // Sin.FUNCTION.apply(scalar.multiply(RealScalar.of(.1))).multiply(RealScalar.of(2.0)), //
    // scalar.multiply(RealScalar.of(0.15))));
    // inside_roundabout()
    ImageRegion imageRegion = R2ImageRegions.inside_circ();
    // ImageRegion asd = new ImageRegion(image, range, outside);
    Region<StateTime> region1 = new R2xTImageStateTimeRegion( //
        imageRegion, rigid1, () -> abstractEntity.getStateTimeNow().time());
    // ---
    // BijectionFamily rigid2 = new Se2Family( //
    // R2xTEllipsoidsAnimationDemo.wrap1DTensor(SimplexContinuousNoise.FUNCTION, Tensors.vector(5, 9, 4), 0.1, 2.0));
    // Tensor polygon = CogPoints.of(4, RealScalar.of(1.5), RealScalar.of(0.5));
    // Region<StateTime> region2 = new R2xTPolygonStateTimeRegion( //
    // polygon, rigid2, () -> abstractEntity.getStateTimeNow().time());
    owlyAnimationFrame.setObstacleQuery(new SimpleTrajectoryRegionQuery( //
        RegionUnion.wrap(Arrays.asList(region1))));
    owlyAnimationFrame.addBackground((RenderInterface) region1);
    owlyAnimationFrame.jFrame.setVisible(true);
  }

  public static void main(String[] args) {
    new R2xTImageAnimationDemo().start();
  }
}
