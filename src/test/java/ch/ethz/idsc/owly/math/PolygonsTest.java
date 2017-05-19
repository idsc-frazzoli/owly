// code by jph
package ch.ethz.idsc.owly.math;

import java.util.ArrayList;
import java.util.List;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class PolygonsTest extends TestCase {
  public void testInside() {
    Tensor polygon = Tensors.matrix(new Number[][] { //
        { 0, 0 }, //
        { 1, 0 }, //
        { 1, 1 }, //
        { 0, 1 } //
    });
    assertTrue(Polygons.isInside(polygon, Tensors.vector(.5, .5)));
    assertTrue(Polygons.isInside(polygon, Tensors.vector(.9, .9)));
    assertTrue(Polygons.isInside(polygon, Tensors.vector(.1, .1)));
    assertFalse(Polygons.isInside(polygon, Tensors.vector(.1, -.1)));
    assertFalse(Polygons.isInside(polygon, Tensors.vector(1, 1.1)));
  }

  public void testSome() {
    Tensor asd = Tensors.vector(2, 3, 4, 5);
    asd.set(RealScalar.of(8), 1);
    System.out.println(asd);
    // asd.get(3).set(tensor, index);
    // Collections.emptyList() = 123;
    List<Integer> list = new ArrayList<>();
    list.add(6);
    list.add(2);
    list.add(3);
    list.add(9);
    list.get(1).longValue();
    {
      String wer = "asdf";
      String wer2 = wer;
      wer = "987345";
      "987345".toString();
      System.out.println(wer);
      System.out.println(wer2);
    }
    {
      Tensor wo = Tensors.vector(2, 3, 4, 5);
      Tensor wo2 = wo;
      wo = Tensors.vector(9, 9);
      System.out.println(wo);
      System.out.println(wo2);
    }
    {
    }
  }
}
