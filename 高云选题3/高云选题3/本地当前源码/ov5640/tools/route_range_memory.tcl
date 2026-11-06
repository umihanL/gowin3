# Re-route the current synthesized netlist; no RTL or constraints are changed.
# Run from gw_sh with optional placement/routing algorithm arguments.
cd [file dirname [file dirname [file normalize [info script]]]]
set started [clock seconds]
set place 1
set route 1
if {$argc > 0} {set place [lindex $argv 0]}
if {$argc > 1} {set route [lindex $argv 1]}
if {[catch {
    open_project ov5640.gprj
    set_option -top_module ov5640_ddr3_tft
    set_option -include_path {src/vga;src/navigation}
    set_option -output_base_name ov5640
    set_option -vcc 0.95
    set_option -use_cpu_as_gpio 1
    set_option -use_sspi_as_gpio 1
    set_option -gen_text_timing_rpt 1
    set_option -place_option $place
    set_option -route_option $route
    run pnr
} message]} {
    puts stderr $message
    exit 1
}
if {![file exists impl/pnr/ov5640.fs] || [file mtime impl/pnr/ov5640.fs] < $started} {
    puts stderr "No new bitstream was generated"
    exit 1
}
set timing_file [open impl/pnr/ov5640.tr r]
set timing_text [read $timing_file]
close $timing_file
foreach kind {Setup Hold} {
    if {![regexp "<Numbers of $kind Violated Endpoints>:(\\d+)" $timing_text match count] || $count != 0} {
        puts stderr "$kind timing failed"
        exit 1
    }
}
exit 0
