// code by jph
package ch.ethz.idsc.owly.demo.rn.glc;

import java.util.Arrays;

import ch.ethz.idsc.owly.demo.rn.R2xTEllipsoidStateTimeRegion;
import ch.ethz.idsc.owly.demo.util.BijectionFamily;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.demo.util.Se2Family;
import ch.ethz.idsc.owly.demo.util.TranslationFamily;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.gui.ani.AbstractEntity;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.math.ScalarTensorFunction;
import ch.ethz.idsc.owly.math.noise.NativeContinuousNoise;
import ch.ethz.idsc.owly.math.noise.SimplexContinuousNoise;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.lie.AngleVector;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

public class R2xTEllipsoidsAnimationDemo implements DemoInterface {
  public static ScalarTensorFunction wrap1DTensor(NativeContinuousNoise nativeContinuousNoise, Tensor offset, double period, double amplitude) {
    return scalar -> Tensor.of(offset.stream().map(Scalar.class::cast)
        .map(value -> RealScalar.of(amplitude * nativeContinuousNoise.at(scalar.number().doubleValue() * period, value.number().doubleValue()))));
  }

  @SuppressWarnings("unused")
  @Override
  public void start() {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    AbstractEntity abstractEntity = new R2xTEntity(Tensors.vector(1.2, 2), RealScalar.of(0.6));
    owlyAnimationFrame.set(abstractEntity);
    // ---
    BijectionFamily shiftx = new TranslationFamily( //
        scalar -> Tensors.of(Sin.FUNCTION.apply(scalar.multiply(RealScalar.of(0.2))), RealScalar.ZERO));
    BijectionFamily shifty = new TranslationFamily( //
        scalar -> Tensors.of(RealScalar.ZERO, //
            Cos.FUNCTION.apply(scalar.multiply(RealScalar.of(0.27)).multiply(RealScalar.of(2)))));
    BijectionFamily circle = new TranslationFamily( //
        scalar -> AngleVector.of(scalar.multiply(RealScalar.of(0.2))).multiply(RealScalar.of(2)));
    BijectionFamily noise = new TranslationFamily( //
        R2xTEllipsoidsAnimationDemo.wrap1DTensor(SimplexContinuousNoise.FUNCTION, Tensors.vector(0, 2), 0.1, 1.3));
    BijectionFamily rigidm = new Se2Family( //
        R2xTEllipsoidsAnimationDemo.wrap1DTensor(SimplexContinuousNoise.FUNCTION, Tensors.vector(5, 9, 4), 0.1, 2.0));
    // ---
    Region<StateTime> region1 = new R2xTEllipsoidStateTimeRegion( //
        Tensors.vector(.7, 0.9), circle, () -> abstractEntity.getStateTimeNow().time());
    Region<StateTime> region2 = new R2xTEllipsoidStateTimeRegion( //
        Tensors.vector(.8, .5), rigidm, () -> abstractEntity.getStateTimeNow().time());
    Region<StateTime> region3 = new R2xTEllipsoidStateTimeRegion( //
        Tensors.vector(.6, .6), noise, () -> abstractEntity.getStateTimeNow().time());
    Region<StateTime> union = RegionUnion.wrap(Arrays.asList(region1, region2, region3));
    owlyAnimationFrame.setObstacleQuery(new SimpleTrajectoryRegionQuery(union));
    owlyAnimationFrame.addBackground((RenderInterface) region1);
    owlyAnimationFrame.addBackground((RenderInterface) region2);
    owlyAnimationFrame.addBackground((RenderInterface) region3);
    owlyAnimationFrame.jFrame.setVisible(true);
  }

  public static void main(String[] args) {
    new R2xTEllipsoidsAnimationDemo().start();
  }
}
