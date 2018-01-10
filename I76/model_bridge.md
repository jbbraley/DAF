The bridge is modeled such that the cross section of the bridge is reduced to a single beam element. The mass per unit length is calculated by the sum of the dead load reactions at girder ends divided by the span length.

Continuity can be controlled via Model Properties>Geometry>Merging

The total reaction force for two continuous 140' spans is 1.810212e+6 lbf.

This is the gravity force associated with the mass of 2 typical 140' span. Therefore, the mass per unit length is 1810/280ft = 6.47 kip/ft.

The beam cross section was set to 1 sq. ft. Therefore, a mass density of 6.47 kip/ft^3 (.0097 slinch/in^3) results in appropriate mass distribution. However, due to the unique geometry of the structure, only a portion of that mass is activated in first bending and thus when shrinking this down to a single girder, we wish only to account for activated mass. In the case of this model, a value of 4.5 kip/ft^3 was found to result in first frequencies in line with what was measured in the field.

The stiffness of the single beam was set such that the displacement under a point load matched that predicted by a full 3-D FE model of a 2-span continuous section. The other spans were set according to stiffness proportions calculated from the different girder sections.


Stiffness proportions for different spans

2-span cont. I11 = 51592.57523406
140' single span = 49787.53215009 (.965X)
111' span  = 47086.20257909 (.913X)

The section stiffness values for the single spans were set by the multiplier times the set stiffness for the 2-span continuous sections.

The bridge has a 3% grade. Models with 3% and no grade were analyzed under a moving bogie (sprung mass) and there was negligeable difference between the resulting bogie and bridge acceleration for the two models. RMS values of the differences between the two models: Truck acceleration = 0.0055 in/sec^2; Bridge acceleration = 0.0012 in/sec^2. Therefore, for further studies, the model with no grade will be used.
