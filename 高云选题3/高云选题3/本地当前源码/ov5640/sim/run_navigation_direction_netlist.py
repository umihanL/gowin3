"""Re-run the fault assertions after the real Gowin synthesis frontend."""
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
source=(ROOT/'sim/run_navigation_visual_netlist.py').read_text()
source=source[:source.index("probe('fusion',")].replace('navigation_visual_','navigation_direction_')
exec(compile(source,str(ROOT/'sim/run_navigation_visual_netlist.py'),'exec'))
probe('fusion','vehicle_vision_fusion',['src/vehicle/vehicle_vision_fusion.v'],
 [(n,[],[]) for n in ('navigation_turn_ownership_tb','navigation_branch_direction_tb',
                     'navigation_next_entry_tb','navigation_handoff_tb','navigation_direction_contract_tb')])
probe('entry','vehicle_visual_localizer',['src/vehicle/vehicle_visual_localizer.v'],
 [('visual_entry_observer_tb',[],[])])
(ROOT/'sim/navigation_direction_netlist_runs.json').write_text(json.dumps(results,indent=2)+'\n')
