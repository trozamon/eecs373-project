#!/bin/bash

rm build.zip

mv power-B_Cu.gbl power.gbl
mv power-B_SilkS.gbo power.gbo
mv power-Edge_Cuts.gbr power.gko
mv power-F_Cu.gtl power.gtl
mv power-F_SilkS.gto power.gto
mv power.drl power.xln
mv power-B_Mask.gbs power.gbs
mv power-B_Paste.gbp power.gbp
mv power-F_Mask.gts power.gts
mv power-F_Paste.gtp power.gtp

zip build.zip power.*
