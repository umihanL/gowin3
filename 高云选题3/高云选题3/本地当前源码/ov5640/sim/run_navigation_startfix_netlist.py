"""Re-use the established Gowin probe runner with this release's test set."""
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
text=(ROOT/'sim/run_navigation_visual_netlist.py').read_text()
text=text[:text.index("probe('fusion',")]
text=text.replace('navigation_visual_', 'navigation_startfix_')
exec(compile(text,str(ROOT/'sim/run_navigation_visual_netlist.py'),'exec'))
probe('fusion','vehicle_vision_fusion',['src/vehicle/vehicle_vision_fusion.v'],
      [(b,[],[]) for b in ('navigation_start_acquire_tb','navigation_branch_direction_tb',
                           'navigation_next_entry_tb','navigation_handoff_tb')])
(ROOT/'sim/navigation_startfix_netlist_runs.json').write_text(json.dumps(results,indent=2)+'\n')
