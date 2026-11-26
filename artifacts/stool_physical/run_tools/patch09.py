"""Diagnostics only: sample the close contact detail into the close-control trace."""
from pathlib import Path
p = Path('/home/user/work/zhaoheng/stool_physical_20261120/toolkits/factory_dual_franka_assembly/plumbers_block_ur5e_skills.py')
t = p.read_text()
old = ("                                        'timing': 'observation at step; command for next physics interval',\n"
       "                                    }\n")
assert t.count(old) == 1
new = ("                                        'timing': 'observation at step; command for next physics interval',\n"
       "                                        # Sparse full contact detail (read-only) for strict-contact review.\n"
       "                                        'contact_detail_sample': (\n"
       "                                            close_detail.get('contact_detail')\n"
       "                                            if close_elapsed_steps % 120 == 0 else None\n"
       "                                        ),\n"
       "                                    },\n"
       "                                    default=str,\n")
t = t.replace(old, new)
# the json.dumps( {...} ) call now has default=str; remove the old closing paren layout check
p.write_text(t)
print('ok')
