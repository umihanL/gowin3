# Run directly in this project; all inputs and outputs stay here.
cd [file dirname [file normalize [info script]]]
set build_started [clock seconds]
if {[catch {
    open_project ov5640.gprj
    set_option -top_module ov5640_ddr3_tft
    set_option -include_path {src/vga;src/navigation}
    set_option -output_base_name ov5640
    set_option -vcc 0.95
    set_option -use_cpu_as_gpio 1
    set_option -use_sspi_as_gpio 1
    set_option -gen_text_timing_rpt 1
    # Try placement algorithm 2 for the nearly full design.
    set_option -place_option 2
    set_option -route_option 0
    # Reuse the previous Arora V placement/routing when available. This is
    # incremental implementation, not partial device reconfiguration.
    if {[info exists ::env(GOWIN_INCREMENTAL_PR)]} {
        set checkpoint [file normalize $::env(GOWIN_INCREMENTAL_PR)]
    } else {
        set checkpoint [file normalize impl/pnr/ov5640.pr]
    }
    if {[file exists impl/incremental_pnr_slow.flag] && ![info exists ::env(GOWIN_INCREMENTAL_PR)]} {
        puts "Prior incremental matching was slower; use standard implementation for this project."
        set_option -inc_pnr 0
    } elseif {[file exists $checkpoint]} {
        puts "NAVIGATION_INCREMENTAL_CHECKPOINT=$checkpoint"
        set_option -inc_pnr $checkpoint
    } else {
        puts "No prior .pr checkpoint; full placement/routing is required."
    }
    run all
} message]} {
    puts stderr $message
    exit 1
}
if {![file exists impl/pnr/ov5640.fs]} {
    puts stderr "Build did not produce impl/pnr/ov5640.fs"
    exit 1
}
if {[file mtime impl/pnr/ov5640.fs] < $build_started} {
    puts stderr "Bitstream is stale: the current build did not regenerate it."
    exit 1
}
set timing_file [open impl/pnr/ov5640.tr r]
set timing_text [read $timing_file]
close $timing_file
foreach kind {Setup Hold} {
    if {![regexp "<Numbers of $kind Violated Endpoints>:(\\d+)" $timing_text match count]} {
        puts stderr "Missing $kind summary in timing report"
        exit 1
    }
    if {$count != 0} {
        puts stderr "$kind timing check failed: $count violated endpoints"
        exit 1
    }
}
exit 0
