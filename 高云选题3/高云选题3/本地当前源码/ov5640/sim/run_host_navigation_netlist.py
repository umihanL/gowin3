"""Check the new UART and turn-entry arbitration after Gowin synthesis."""
from pathlib import Path
import hashlib, json, re, subprocess, time

ROOT = Path(__file__).resolve().parents[1]
GOWIN = 'C:/Gowin/Gowin_V1.9.12.03_x64/IDE'
PRIM = GOWIN + '/simlib/gw5a/prim_sim.v'
results = []

def execute(commands, log):
    with (ROOT/log).open('w', encoding='utf-8') as output:
        for command in commands:
            p = subprocess.run(command, cwd=ROOT, stdout=output,
                               stderr=subprocess.STDOUT, timeout=400)
            if p.returncode:
                raise RuntimeError(f'exit {p.returncode}: {log}')

def probe(name, top, sources, benches, wrapper=None):
    folder = ROOT/f'impl/host_{name}_probe'
    folder.mkdir(exist_ok=True)
    if wrapper:
        (folder/'wrapper.v').write_text(wrapper)
        sources = [*sources, str((folder/'wrapper.v').relative_to(ROOT))]
    files = ''.join(f'<File path="{(ROOT/p).as_posix()}" type="file.verilog" enable="1" />' for p in sources)
    project = re.sub(r'<FileList>.*?</FileList>', '<FileList>'+files+'</FileList>',
                     (ROOT/'ov5640.gprj').read_text(), flags=re.S)
    (folder/'probe.gprj').write_text(project)
    (folder/'build.tcl').write_text(
        'cd [file dirname [file normalize [info script]]]\nopen_project probe.gprj\n'
        f'set_option -include_path {{{ROOT.as_posix()}}}\n'
        f'set_option -top_module {top}\nset_option -output_base_name probe\n'
        'if {[catch {run syn} message]} {puts stderr $message; exit 1}\nexit 0\n')
    execute([[GOWIN+'/bin/gw_sh.exe', str(folder/'build.tcl')]], f'sim/host_{name}_probe_synthesis.log')
    for bench, extra, replace in benches:
        text = (ROOT/f'sim/{bench}.sv').read_text()
        text = text.replace(f'module {bench};', f"module {bench};\n GSR GSR(.GSRI(1'b1));")
        for old, new in replace:
            assert old in text, old
            text = text.replace(old, new)
        target = folder/(bench+'.sv');target.write_text(text)
        log = f'sim/host_{name}_{bench}_netlist.log'
        output = f'sim/host_{name}_{bench}_netlist.vvp'
        started = time.monotonic()
        execute([['iverilog', '-g2012', '-s', bench, '-o', output, str(target),
                  str(folder/'impl/gwsynthesis/probe.vg'), PRIM, *extra], ['vvp', output]], log)
        content = (ROOT/log).read_text()
        assert 'PASS ' in content and '$finish called' in content, log
        results.append(dict(name=name+'_'+bench, passed=True, seconds=round(time.monotonic()-started,3),
                            log=log, sha256=hashlib.sha256((ROOT/log).read_bytes()).hexdigest(),
                            inputs={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest()
                                    for p in sources+[f'sim/{bench}.sv']}))
        print('PASS synthesized', name, bench, flush=True)

probe('fusion', 'vehicle_vision_fusion', ['src/vehicle/vehicle_vision_fusion.v'],
      [(b, [], []) for b in ('navigation_next_entry_tb', 'navigation_handoff_tb')])
wrapper = '''module host_probe(
 input clk,rst_n,rx,selected,drive_page,navigation_page,navigation_ready,
 input [3:0] motion_status,motion_phase,
 input event_valid,input [7:0] event_code,input [15:0] event_value,
 output tx,run,nav_session,output [15:0] speed_limit,
 output link_ok,output [7:0] task_id
);
 vehicle_host_link #(.CLK_HZ(1000000),.BAUD(100000),.LINK_TIMEOUT_MS(200)) core(
  .clk(clk),.rst_n(rst_n),.rx(rx),.selected(selected),.drive_page(drive_page),
  .navigation_page(navigation_page),.navigation_ready(navigation_ready),
  .motion_status(motion_status),.motion_phase(motion_phase),.event_valid(event_valid),
  .event_code(event_code),.event_value(event_value),.tx(tx),.run(run),
  .nav_session(nav_session),.speed_limit(speed_limit),.link_ok(link_ok),.task_id(task_id));
endmodule
'''
monitor=ROOT/'sim/host_uart_monitor.v'
monitor.write_text((ROOT/'src/vehicle/host_uart_rx.v').read_text().replace('module host_uart_rx #','module host_uart_monitor #'))
probe('uart', 'host_probe', ['src/vehicle/vehicle_host_link.v', 'src/vehicle/host_uart_rx.v',
      'src/vehicle/motor/uart_byte_tx.v'], [('vehicle_host_link_tb', ['sim/host_uart_monitor.v'],
      [('vehicle_host_link #(.CLK_HZ(CLK_HZ),.BAUD(BAUD),.LINK_TIMEOUT_MS(200)) dut(', 'host_probe dut('),
       ('host_uart_rx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) monitor(', 'host_uart_monitor #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) monitor('),
       ('bytes,dut.pos', "bytes,4'd0")])], wrapper)
(ROOT/'sim/host_navigation_netlist_runs.json').write_text(json.dumps(results, indent=2)+'\n')
