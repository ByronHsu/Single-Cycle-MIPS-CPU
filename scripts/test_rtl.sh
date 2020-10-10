echo "Baseline"
ncverilog tb.v SingleCycleMIPS.v +define+Baseline +access+r
echo "FPU-Baseline"
ncverilog tb.v SingleCycleMIPS_FPU.v +define+Baseline +access+r -y /usr/cad/synopsys/synthesis/cur/dw/sim_ver/ +libext+.v +incdir+/usr/cad/synopsys/synthesis/cur/dw/sim_ver/
echo "FPU-Single"
ncverilog tb.v SingleCycleMIPS_FPU.v +define+FPU+Single +access+r -y /usr/cad/synopsys/synthesis/cur/dw/sim_ver/ +libext+.v +incdir+/usr/cad/synopsys/synthesis/cur/dw/sim_ver/
echo "FPU-Double"
ncverilog tb.v SingleCycleMIPS_FPU.v +define+FPU+Double +access+r -y /usr/cad/synopsys/synthesis/cur/dw/sim_ver/ +libext+.v +incdir+/usr/cad/synopsys/synthesis/cur/dw/sim_ver/
