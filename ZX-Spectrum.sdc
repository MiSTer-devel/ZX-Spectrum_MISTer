derive_pll_clocks
derive_clock_uncertainty

set clk_sys {*|pll|pll_inst|altera_pll_i|*[0].*|divclk}
set clk_56m {*|pll|pll_inst|altera_pll_i|*[1].*|divclk}

set_multicycle_path -from [get_clocks $clk_56m] -to [get_clocks $clk_sys] -setup 2
set_multicycle_path -from [get_clocks $clk_56m] -to [get_clocks $clk_sys] -hold 1

# Effective clock is only half of the system clock, so allow 2 clock cycles for the paths in the T80 cpu
set_multicycle_path -from {emu|cpu|u0|*} -setup 2
set_multicycle_path -from {emu|cpu|u0|*} -hold 1

# The CE is only active in every 2 clocks, so allow 2 clock cycles
set_multicycle_path -from {emu|tape|*} -setup 2
set_multicycle_path -from {emu|tape|*} -hold 1

set_multicycle_path -from {emu|wd1793|sbuf|*} -setup 2
set_multicycle_path -from {emu|wd1793|sbuf|*} -hold 1
set_multicycle_path -from {emu|wd1793|edsk_rtl_0|*} -setup 2
set_multicycle_path -from {emu|wd1793|edsk_rtl_0|*} -hold 1
set_multicycle_path -from {emu|wd1793|layout_r*} -setup 2
set_multicycle_path -from {emu|wd1793|layout_r*} -hold 1
set_multicycle_path -from {emu|wd1793|disk_track*} -setup 2
set_multicycle_path -from {emu|wd1793|disk_track*} -hold 1
set_multicycle_path -to   {emu|wd1793|state[*]} -setup 2
set_multicycle_path -to   {emu|wd1793|state[*]} -hold 1
set_multicycle_path -to   {emu|wd1793|wait_time[*]} -setup 2
set_multicycle_path -to   {emu|wd1793|wait_time[*]} -hold 1

set_false_path -to {emu|wd1793|s_seekerr}

set_multicycle_path -from {emu|u765|sbuf|*} -setup 2
set_multicycle_path -from {emu|u765|sbuf|*} -hold 1
set_multicycle_path -from {emu|u765|image_track_offsets_rtl_0|*} -setup 2
set_multicycle_path -from {emu|u765|image_track_offsets_rtl_0|*} -hold 1
set_multicycle_path -to   {emu|u765|i_*} -setup 2
set_multicycle_path -to   {emu|u765|i_*} -hold 1
set_multicycle_path -to   {emu|u765|i_*[*]} -setup 2
set_multicycle_path -to   {emu|u765|i_*[*]} -hold 1
set_multicycle_path -to   {emu|u765|pcn[*]} -setup 2
set_multicycle_path -to   {emu|u765|pcn[*]} -hold 1
set_multicycle_path -to   {emu|u765|ncn[*]} -setup 2
set_multicycle_path -to   {emu|u765|ncn[*]} -hold 1
set_multicycle_path -to   {emu|u765|state[*]} -setup 2
set_multicycle_path -to   {emu|u765|state[*]} -hold 1
set_multicycle_path -to   {emu|u765|status[*]} -setup 2
set_multicycle_path -to   {emu|u765|status[*]} -hold 1
set_multicycle_path -to   {emu|u765|i_rpm_time[*][*][*]} -setup 8
set_multicycle_path -to   {emu|u765|i_rpm_time[*][*][*]} -hold 7

set_multicycle_path -from {emu|load} -setup 2
set_multicycle_path -from {emu|load} -hold 1

set_false_path -from {emu|init_reset}
set_false_path -from {emu|hps_io|cfg*}
set_false_path -from {emu|hps_io|status*}
set_false_path -from {emu|arch_reset}
set_false_path -from {emu|snap_loader|snap_reset}
set_false_path -from {emu|kbd|Fn*}
set_false_path -from {emu|kbd|mod*}
set_false_path -from {emu|plus3}
set_false_path -from {emu|zx48}
set_false_path -from {emu|p1024}
set_false_path -from {emu|pf1024}
