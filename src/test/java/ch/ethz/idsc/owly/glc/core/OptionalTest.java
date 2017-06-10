// code by jph
package ch.ethz.idsc.owly.glc.core;

import java.util.Optional;

import junit.framework.TestCase;

public class OptionalTest extends TestCase {
  public void testSimple() {
    Optional<String> asd = Optional.ofNullable(null);
    assertFalse(asd.isPresent());
  }

  public void testSimple2() {
    Optional<String> asd = Optional.ofNullable("asdfasdf");
    assertTrue(asd.isPresent());
  }
}
