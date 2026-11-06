# Keep the fresh synthesis result; recover from slow incremental matching.
cd [file dirname [file dirname [file normalize [info script]]]]
set started [clock seconds]
if {[catch {
    open_project ov5640.gprj
    set_option -top_module ov5640_ddr3_tft
    set_option -include_path {src/vga;src/navigation}
    set_option -output_base_name ov5640
    set_option -vcc 0.95
    set_option -use_cpu_as_gpio 1
    set_option -use_sspi_as_gpio 1
    set_option -gen_text_timing_rpt 1
    set_option -place_option 2
    set_option -route_option 0
    set_option -inc_pnr 0
    set_option -inc_place 0
    puts "NAVIGATION_FALLBACK_REUSE_SYNTHESIS=1"
    run pnr
} message]} {puts stderr $message;exit 1}
if {![file exists impl/pnr/ov5640.fs] || [file mtime impl/pnr/ov5640.fs] < $started} {
    puts stderr "No fresh bitstream was generated";exit 1
}
set f [open impl/pnr/ov5640.tr r]
set timing [read $f]
close $f
foreach kind {Setup Hold} {
    if {![regexp "<Numbers of $kind Violated Endpoints>:(\\d+)" $timing match count] || $count!=0} {
        puts stderr "$kind timing failed";exit 1
    }
}
exit 0
