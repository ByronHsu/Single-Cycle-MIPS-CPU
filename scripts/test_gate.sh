echo "Baseline"
ncverilog tb.v SingleCycleMIPS_syn.v tsmc13.v +define+Baseline+SDF +access+r
echo "FPU-Single"
ncverilog tb.v SingleCycleMIPS_FPU_syn.v tsmc13.v +define+FPU+Single +access+r
echo "FPU-Double"
ncverilog tb.v SingleCycleMIPS_FPU_syn.v tsmc13.v +define+FPU+Double +access+r