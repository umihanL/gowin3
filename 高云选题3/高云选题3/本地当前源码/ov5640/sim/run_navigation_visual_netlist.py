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
    folder = ROOT/f'impl/navigation_visual_{name}_probe'
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
    execute([[GOWIN+'/bin/gw_sh.exe', str(folder/'build.tcl')]], f'sim/navigation_visual_{name}_probe_synthesis.log')
    for bench, extra, replace in benches:
        text = (ROOT/f'sim/{bench}.sv').read_text()
        text = text.replace(f'module {bench};', f"module {bench};\n GSR GSR(.GSRI(1'b1));")
        for old, new in replace:
            assert old in text, old
            text = text.replace(old, new)
        target = folder/(bench+'.sv');target.write_text(text)
        log = f'sim/navigation_visual_{name}_{bench}_netlist.log'
        output = f'sim/navigation_visual_{name}_{bench}_netlist.vvp'
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
      [(b, [], []) for b in ('navigation_next_entry_tb', 'navigation_handoff_tb','navigation_branch_direction_tb')])
probe('localizer','vehicle_visual_localizer',['src/vehicle/vehicle_visual_localizer.v'],
      [('visual_localizer_tb',[],[])])
(ROOT/'sim/navigation_visual_netlist_runs.json').write_text(json.dumps(results,indent=2)+'\n')
