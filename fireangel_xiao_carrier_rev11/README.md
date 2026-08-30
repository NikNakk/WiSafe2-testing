# Rev 11 review fixes

This revision addresses review of the generated KiCad file itself.

## Fixed: explicit KiCad net table and numeric net codes

The board now contains an explicit top-level net table:

- 1 GND
- 2 +3V3
- 3 CS
- 4 IRQ
- 5 MOSI
- 6 SCK
- 7 MISO
- 8 ANT

Connected through-hole pads use `(net N "NAME")`, tracks/vias use `(net N)`,
and zones use `(net N) (net_name "NAME")`.

## Review comments about CS/IRQ short and dangling stubs

Those coordinates do **not** occur in the current Rev 10/Rev 11 board. They
appear to describe an older generated layout. The current routing was inspected
before this conversion and does not contain the cited `(15,31)`, `(15,35)`,
`(15,37.54)`, `(15,39)`, `(15,43)`, `(15,47)`, `(33,42.5)`, or `(7,51)` stubs.

There are no routing changes from Rev 10 in Rev 11; this revision changes only
the KiCad net representation.

Please open in KiCad 10.0.5 and run DRC again before fabrication.
