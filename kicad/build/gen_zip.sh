#!/bin/bash

rm build.zip

mv power-B_Cu.gbl power.gbl
mv power-B_SilkS.gbo power.gbo
mv power-Edge_Cuts.gbr power.gko
mv power-F_Cu.gtl power.gtl
mv power-F_SilkS.gto power.gto
mv power.drl power.xln

zip build.zip power.gbl power.gbo power.gko power.gtl power.gto power.xln
