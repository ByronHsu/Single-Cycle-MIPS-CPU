set cycle  50      ;#clock period defined by designer

##########################################
#  you cannot change the following parts
##########################################

create_clock -period $cycle [get_ports  clk]
set_dont_touch_network      [get_clocks clk]
set_fix_hold                [get_clocks clk]
set_ideal_network           [get_ports clk]
set_clock_uncertainty  0.1  [get_clocks clk]
set_clock_latency      0.5  [get_clocks clk]
set_input_transition   0.5  [all_inputs]
set_clock_transition   0.1  [all_clocks]

set_input_delay  0.5    -clock clk [remove_from_collection [all_inputs] [get_ports clk]]
set_output_delay 0.5    -clock clk [all_outputs] 
set_load         0.1     [all_outputs]
set_drive        1     [all_inputs]

set_operating_conditions  -max slow  -min fast
set_wire_load_model -name tsmc13_wl10 -library slow                        

set_max_fanout 20 [all_inputs]