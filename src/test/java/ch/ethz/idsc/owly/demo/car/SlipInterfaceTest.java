// code by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.pdf.Distribution;
import ch.ethz.idsc.tensor.pdf.NormalDistribution;
import ch.ethz.idsc.tensor.pdf.RandomVariate;
import ch.ethz.idsc.tensor.sca.Chop;
import junit.framework.TestCase;

public class SlipInterfaceTest extends TestCase {
  public void testSimple() {
    CHatchbackModel c = new CHatchbackModel();
    new RobustSlip(c.pacejka1(), Tensors.vector(0, 0), RealScalar.ZERO).slip();
    try {
      new TextbookSlip(c.pacejka1(), Tensors.vector(0, 0), RealScalar.ZERO).slip();
      assertTrue(false);
    } catch (Exception exception) {
      // ---
    }
  }

  public void testEquality1() {
    CHatchbackModel c = new CHatchbackModel();
    SlipInterface si1 = new RobustSlip(c.pacejka1(), Tensors.vector(1, 0), RealScalar.of(1));
    SlipInterface si2 = new TextbookSlip(c.pacejka1(), Tensors.vector(1, 0), RealScalar.of(1));
    assertEquals(si1.slip(), si2.slip());
  }

  public void testEquality2() {
    CHatchbackModel c = new CHatchbackModel();
    Distribution distribution = NormalDistribution.standard();
    for (int index = 0; index < 100; ++index) {
      Scalar rtw = RandomVariate.of(distribution);
      SlipInterface si1 = new RobustSlip(c.pacejka1(), Tensors.vector(1, 0), rtw);
      SlipInterface si2 = new TextbookSlip(c.pacejka1(), Tensors.vector(1, 0), rtw);
      assertTrue(Chop.isZeros(si1.slip().subtract(si2.slip())));
    }
  }

  public void testEquality3() {
    CHatchbackModel c = new CHatchbackModel();
    Distribution distribution = NormalDistribution.standard();
    for (int index = 0; index < 100; ++index) {
      Scalar vx = RandomVariate.of(distribution);
      Scalar vy = RandomVariate.of(distribution);
      Scalar rtw = RandomVariate.of(distribution);
      SlipInterface si1 = new RobustSlip(c.pacejka1(), Tensors.of(vx, vy), rtw);
      SlipInterface si2 = new TextbookSlip(c.pacejka1(), Tensors.of(vx, vy), rtw);
      assertTrue(Chop.isZeros(si1.slip().subtract(si2.slip())));
    }
  }
}
