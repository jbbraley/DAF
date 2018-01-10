## Areas that still require research:

1. Dynamic effects on service limit states. (i.e. cyclic strain in deck causing cracking)
2. Optimize vehicle speed and spacing on multiple lanes via FE
3. Effect of bridge geometry on DAF: 2-span cont.
4. Effect of truck caravans
5. Population FE analysis of traffic dynamic loading

## Question: How is vibration on one bridge (side) able to be felt by the other side
1. Becuase it is transmitting a lot of vibration
  1. The two sides are connected by several mechanisms that are capable of transferring force/motion
  2. The modes which excite those mechanism will be capable of transmitting vibrations, and will be a possible mode to be excited
    1. Look for modes for which both sides are "displacing"
    2. Run Transient analysis, fft-> which modes are excited in the non-loaded span (phases)
  3. Isolate and identify mode shapes by analyzing the sides separately for pole selection of the combined cmif
