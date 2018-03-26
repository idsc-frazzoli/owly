// code by jph
package ch.ethz.idsc.owly.demo.rn;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Path2D;
import java.util.function.Supplier;

import ch.ethz.idsc.owl.gui.GeometricLayer;
import ch.ethz.idsc.owl.math.map.BijectionFamily;
import ch.ethz.idsc.owly.demo.util.RegionRenders;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.opt.TensorUnaryOperator;

/** visualize planar ellipse */
public class R2xTEllipsoidConstraintRegion extends R2xTEllipsoidStateTimeRegion {
  public R2xTEllipsoidConstraintRegion(Tensor radius, BijectionFamily bijectionFamily, Supplier<Scalar> supplier) {
    super(radius, bijectionFamily, supplier);
    // TODO Auto-generated constructor stub
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    Scalar time = supplier.get();
    TensorUnaryOperator fwd = bijectionFamily.forward(time);
    Path2D path2D = geometricLayer.toPath2D(Tensor.of(polygon.stream().map(fwd)));
    graphics.setColor(new Color(255, 205, 186));
    graphics.fill(path2D);
    graphics.setColor(RegionRenders.BOUNDARY);
    path2D.closePath();
    graphics.draw(path2D);
  }
}
