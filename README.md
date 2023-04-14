# MIG_weldong
dong to weld with MIG
Dreamed by BingAI

use creativity to create Arduino implementation of new type of MIG arc welding controller. It can use any combination of analog inputs, controls provided by analog inputs, feedback like voltage and current. Controls should include target wire feed rate, and wire burn pulse fudge factor. as output it should control PWM with inductor and capacitor (dc-dc buck circuit) and step/dir controlled stepper motor for wire feed. As strategy it should feed the wire until contact while using small current to initiate arc, retract the wire after contact while rapidly increasing output voltage and current so the arc is sustained for a short pulse of duration estimated to melt fragment of a wire, then lower the constant current level to 20% , feed the wire by 50% of estimated wire burn amount by the previous pulse and perform arc voltage measurement to estimate how much wire got actually burnt . Then feed the corrected estimate of the wire while performing another burn pulse and repeat measurement cycle converging to make process repeating and predictable. Be creative in inventing wire feed strategies, use concepts like stepper motor acceleration and jerk and try to make the feed process oscillating and converging with wire burn pulse process and produce least wire burn amount per pulse error.
