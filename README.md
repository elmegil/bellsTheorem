# bellsTheorem
Bell's theorem is a proof that quantum physics is incompatible with local hidden variable theories.  The summary on Wikipedia is quite good if you’re interested in the quantum physics aspects of the theorem.

The proof is sometimes demonstrated by the use of multiple polarized lenses, as in this video:

https://www.youtube.com/watch?v=zcqZHYo7ONs&t=337s

What’s interesting is that with 3 or more polarized lenses, the behavior does not match what you might expect, namely that lens 1 reduces the light by a certain percentage, lens 2 reduces it further, and lens 3 reduces it even further, in a linear fashion.  If lens 3 and lens 1 are 90 degrees from each other, with nothing else involved, essentially no light gets through.  But if you insert lens 2 at a 45 degree angle to both of the others, suddenly there IS light getting through.  You’d expect that if 3 after 1 always blocks all the light, that would be true no matter what you put between the two.

A highly simplified explanation of this has to do with the fact that light is made up of photons, which are quantum objects.  Quantum objects behave probabilistically but ultimately are either there or not.  Photons going from lens 1 to lens 3 aren’t reduced in energy, they simply don’t pass both lenses.  However, going from one lens to one at 45 degrees only reduces the probability of photos passing by 50%.  In the process of going through, those photons then match the orientation of the second lens.  So when they get to a third lens, there is only a 45 degree angle difference there as well, and so 50% of the original 50%, or 25% of the original light, still passes through.

The rules for photons passing through polarized lenses basically are a probability that is determined entirely by the two lenses directly adjacent to the path of travel.  Any previous or subsequent lenses  aren’t considered except as a chained operation.  So with lens 1 at 0 degrees, lens 2 at 45, and lens 3 at 90 degrees (or 45 degrees from lens 2), the overall probability of a photon going from the start to the finish is 50% x 50%, or 25%.

Making things slightly weirder, the probabilities aren’t linear.  They follow a cosine squared relationship.  A photon passing through a pair of lenses at 22.5 degrees from each other is considerably more likely to pass through than the linear expectation of 75%.  In fact it’s about 85%.

============================================================================

Nyles Miszczyk on Facebook asked whether there were any modules that could process control voltages in this way.  There do not appear to be, but the idea was intriguing enough to give it a shot.  And that’s where this module comes from.



# Operational Notes

There is a single input voltage to be processed, which is normalled to +12V.  There are 4 encoders which represent the relative positions of four lenses, first to a “main” lens behind them all, and then relative to each other.  They’re arranged in a square pattern, and the input “light” (really the voltage) is processed as it goes through a single lens at 0 degrees, and then each subsequent lens.  Each of the combinations of lenses has a related output, so each of A, B, C, and D have an output that is only related to their individual angles relative to 0.  Each of AB, BC, CD, and DA has an output related to each of the intervening angles, as in the example described above; so for AB the voltage is reduced by whatever the probability is between 0 degrees and angle A, and then further reduced by whatever the probability is between the angles of A and B.  ABC, BCD, CDA, and DAB then follow the same rules for three lenses, and finally ABCD extends the process through all 4 lenses.

In addition to the encoders, each of the 4 lenses has a CV that is also related to its angle, added to the angle as set up by the encoder.  The 0 - 10V range maps to 180 degrees of rotation (positive only… need to consider doing otherwise).

On first power up, the lenses are reset to 0 degrees (but they should retain the last known state).  You can press each encoder to reset it to zero so that it’s at a known state.  If you do this by accident, pressing the encoder again takes you back to the saved state.  HOWEVER, if you zero out this way and then change things, you do do NOT save the new angle when you press again -- you just go back to the saved state, and pressing one more time takes you back to zero.

Voltage ranges: the current implementation limits inputs to 0-10V and each output will only generate 0-10V.  (and even that not very precisely).  A future revision may allow processing of bipolar input voltages, although this very likely will reduce the overall resolution.  Each input should be able to tolerate any normal Eurorack voltages outside this range (-12V to +12V), but they will be clamped at 0-10V.

Encoders:  the encoders’ positions are NOT directly mapped to the angle.  Most encoders only have about 24 steps (some have more, but none have 360 steps).  That did not seem to be a good resolution because each step would be 15 degrees.  I have mapped the encoders so that each step is 2 degrees of angle; this is why the press to reset is important so you can get back to a known angle. -- need to decide about CVs… 0 - 10V … is that 0 - 90 degrees?  0 - 180?  0 - 360?  Maybe consider 30 step encoders that do map the angle?  Is there any way to set up to display the angle, circle of LEDs on each encoder?  Is there sufficient space/bandwidth to update them?
