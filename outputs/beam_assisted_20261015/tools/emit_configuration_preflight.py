"""Inspect real randomized episode configs without creating a simulator."""
from pathlib import Path
import json
import sys
from toolkits.factory_dual_franka_assembly.scene_builder import build_dual_franka_assembly_episode
from toolkits.factory_dual_franka_assembly.beam_assisted_preflight import assert_assisted_configuration

rows = []
for layout in [0, 1, 505, 777]:
    cfg = build_dual_franka_assembly_episode(
        recipe='fabrica_beam_ur5e_assisted', scene_profile='taoyuan_grscenes_tabletop',
        seed=0, layout_seed=layout, domain_randomization_enabled=True)
    report = assert_assisted_configuration(cfg)
    if layout == 505:
        Path(sys.argv[1]).with_name(Path(sys.argv[1]).stem+'_layout505_config.json').write_text(
            json.dumps(cfg.model_dump(mode='python',serialize_as_any=True),indent=2)+'\n')
    rows.append({'seed': 0, 'layout_seed': layout, 'report': report,
                 'max_steps': cfg.max_steps, 'phase_count': len(cfg.phase_specs),
                 'wrist_cameras': [c for c in cfg.camera_metadata
                                   if c.get('target_source') == 'native_fingertip_midpoint']})
Path(sys.argv[1]).write_text(json.dumps({'passed': True, 'simulator_created': False,
    'scope': 'actual randomized five-part episode construction and production target resolver; no native motion or image certification',
    'rows': rows}, indent=2) + '\n')
print(json.dumps({'passed': True, 'layouts': len(rows), 'phase_count': rows[0]['phase_count'],
                  'compiled_max_steps': rows[0]['max_steps'], 'simulator_created': False}))
