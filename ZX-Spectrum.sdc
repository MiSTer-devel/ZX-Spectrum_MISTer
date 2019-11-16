
# Effective clock is only half of the system clock, so allow 2 clock cycles for the paths in the T80 cpu
set_multicycle_path -from {emu|cpu|u0|*} -setup 2
set_multicycle_path -from {emu|cpu|u0|*} -hold 1

set_multicycle_path -from {emu|gs|cpu|u0|*} -setup 2
set_multicycle_path -from {emu|gs|cpu|u0|*} -hold 1


# The CE is only active in every 2 clocks, so allow 2 clock cycles
set_multicycle_path -to {emu:emu|smart_tape:tape|tape:tape|size[*]} -setup 2
set_multicycle_path -to {emu:emu|smart_tape:tape|tape:tape|size[*]} -hold 1
set_multicycle_path -to {emu:emu|smart_tape:tape|tape:tape|read_cnt[*]} -setup 2
set_multicycle_path -to {emu:emu|smart_tape:tape|tape:tape|read_cnt[*]} -hold 1
set_multicycle_path -to {emu:emu|smart_tape:tape|tape:tape|blocksz[*]} -setup 2
set_multicycle_path -to {emu:emu|smart_tape:tape|tape:tape|blocksz[*]} -hold 1
set_multicycle_path -to {emu:emu|smart_tape:tape|tape:tape|timeout[*]} -setup 2
set_multicycle_path -to {emu:emu|smart_tape:tape|tape:tape|timeout[*]} -hold 1
set_multicycle_path -to {emu:emu|smart_tape:tape|tape:tape|pilot[*]} -setup 2
set_multicycle_path -to {emu:emu|smart_tape:tape|tape:tape|pilot[*]} -hold 1
set_multicycle_path -to {emu:emu|smart_tape:tape|tape:tape|tick[*]} -setup 2
set_multicycle_path -to {emu:emu|smart_tape:tape|tape:tape|tick[*]} -hold 1
set_multicycle_path -to {emu:emu|smart_tape:tape|tape:tape|blk_list[*]} -setup 2
set_multicycle_path -to {emu:emu|smart_tape:tape|tape:tape|blk_list[*]} -hold 1
set_multicycle_path -to {emu:emu|smart_tape:tape|tape:tape|bitcnt[*]} -setup 2
set_multicycle_path -to {emu:emu|smart_tape:tape|tape:tape|bitcnt[*]} -hold 1
set_multicycle_path -from {emu:emu|smart_tape:tape|tape:tape|rd_req} -setup 2
set_multicycle_path -from {emu:emu|smart_tape:tape|tape:tape|rd_req} -hold 1

# The effective clock fo the AY chips are 112/1.75=64 cycles, so allow at least 2 cycles for the paths
set_multicycle_path -to {emu:emu|turbosound:turbosound|*} -setup 4
set_multicycle_path -to {emu:emu|turbosound:turbosound|*} -hold 3
set_multicycle_path -to {emu:emu|turbosound:turbosound|jt03:ym2203_0|*} -setup 4
set_multicycle_path -to {emu:emu|turbosound:turbosound|jt03:ym2203_0|*} -hold 3
set_multicycle_path -to {emu:emu|turbosound:turbosound|jt03:ym2203_1|*} -setup 4
set_multicycle_path -to {emu:emu|turbosound:turbosound|jt03:ym2203_1|*} -hold 3

set_multicycle_path -to {emu:emu|saa1099:saa1099|*} -setup 8
set_multicycle_path -to {emu:emu|saa1099:saa1099|*} -hold 7

set_multicycle_path -from {emu:emu|wd1793:fdd|wd1793_dpram:sbuf|*} -setup 2
set_multicycle_path -from {emu:emu|wd1793:fdd|wd1793_dpram:sbuf|*} -hold 1
set_multicycle_path -from {emu:emu|wd1793:fdd|altsyncram:edsk_rtl_0|*} -setup 2
set_multicycle_path -from {emu:emu|wd1793:fdd|altsyncram:edsk_rtl_0|*} -hold 1
set_multicycle_path -from {emu:emu|wd1793:fdd|layout_r*} -setup 2
set_multicycle_path -from {emu:emu|wd1793:fdd|layout_r*} -hold 1
set_multicycle_path -from {emu:emu|wd1793:fdd|disk_track*} -setup 2
set_multicycle_path -from {emu:emu|wd1793:fdd|disk_track*} -hold 1

set_multicycle_path -to {emu:emu|wd1793:fdd|state[*]} -setup 2
set_multicycle_path -to {emu:emu|wd1793:fdd|state[*]} -hold 1
set_multicycle_path -to {emu:emu|wd1793:fdd|wait_time[*]} -setup 2
set_multicycle_path -to {emu:emu|wd1793:fdd|wait_time[*]} -hold 1

set_multicycle_path -from {emu:emu|u765:u765|u765_dpram:sbuf|*} -setup 2
set_multicycle_path -from {emu:emu|u765:u765|u765_dpram:sbuf|*} -hold 1
set_multicycle_path -from {emu:emu|u765:u765|altsyncram:image_track_offsets_rtl_0|*} -setup 2
set_multicycle_path -from {emu:emu|u765:u765|altsyncram:image_track_offsets_rtl_0|*} -hold 1
set_multicycle_path -to {emu:emu|u765:u765|i_*} -setup 2
set_multicycle_path -to {emu:emu|u765:u765|i_*} -hold 1
set_multicycle_path -to {emu:emu|u765:u765|i_*[*]} -setup 2
set_multicycle_path -to {emu:emu|u765:u765|i_*[*]} -hold 1
set_multicycle_path -to {emu:emu|u765:u765|pcn[*]} -setup 2
set_multicycle_path -to {emu:emu|u765:u765|pcn[*]} -hold 1
set_multicycle_path -to {emu:emu|u765:u765|ncn[*]} -setup 2
set_multicycle_path -to {emu:emu|u765:u765|ncn[*]} -hold 1
set_multicycle_path -to {emu:emu|u765:u765|state[*]} -setup 2
set_multicycle_path -to {emu:emu|u765:u765|state[*]} -hold 1
set_multicycle_path -to {emu:emu|u765:u765|status[*]} -setup 2
set_multicycle_path -to {emu:emu|u765:u765|status[*]} -hold 1
set_multicycle_path -to {emu:emu|u765:u765|i_rpm_time[*][*][*]} -setup 8
set_multicycle_path -to {emu:emu|u765:u765|i_rpm_time[*][*][*]} -hold 7

set_false_path -from {emu|init_reset}
set_false_path -from {emu|hps_io|cfg*}
set_false_path -from {emu|hps_io|status*}
set_false_path -from {emu|arch_reset}
set_false_path -from {emu|snap_loader|snap_reset}
set_false_path -from {emu|kbd|Fn*}
set_false_path -from {emu|kbd|mod*}
set_false_path -from {emu|plus3}
