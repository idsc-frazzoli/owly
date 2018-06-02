// code by ynager
package ch.ethz.idsc.owly.demo.util;

import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class LexicographicSlackTest extends TestCase {
  public void testOne() {
    Tensor a;
    Tensor b;
    int comp;
    // ---
    Tensor slacks = Tensors.vector(0.1, 0.1, 0.1);
    LexicographicSlack c = LexicographicSlack.of(slacks);
    // ---
    a = Tensors.vector(111, 0, 0);
    b = Tensors.vector(100, 500, 500);
    comp = c.compare(a, b);
    assertEquals(comp, Integer.compare(1, 0));
    // ---
    a = Tensors.vector(110, 0, 0);
    b = Tensors.vector(100, 500, 500);
    comp = c.compare(a, b);
    assertEquals(comp, Integer.compare(0, 1));
    // ---
    a = Tensors.vector(110, 500, 500);
    b = Tensors.vector(100, 500, 500);
    comp = c.compare(a, b);
    assertEquals(comp, Integer.compare(0, 0));
    // ---
    a = Tensors.vector(96, 95, 111);
    b = Tensors.vector(100, 100, 100);
    comp = c.compare(a, b);
    assertEquals(comp, Integer.compare(1, 0));
    // ---
    a = Tensors.vector(100, 0, 0.1);
    b = Tensors.vector(100, 0, 0);
    comp = c.compare(a, b);
    assertEquals(comp, Integer.compare(1, 0));
    // ---
  }

  public void testMore() {
    Tensor slacks = Tensors.vector(0.1, 0, 0.1);
    LexicographicSlack c = LexicographicSlack.of(slacks);
    Tensor a = Tensors.vector(95, 100.01, 0);
    Tensor b = Tensors.vector(100, 100, 100);
    int comp = c.compare(a, b);
    assertEquals(comp, Integer.compare(1, 0));
  }
}
