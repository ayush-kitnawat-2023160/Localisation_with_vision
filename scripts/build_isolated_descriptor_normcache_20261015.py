#!/usr/bin/env python3
"""Make an A/B image differing from the frozen healthy baseline ONLY in matcher.

This avoids unintentionally including unrelated in-flight changes from the
working source tree. No production image or default launch is overwritten.
"""
from pathlib import Path
import difflib
import subprocess

root=Path(__file__).resolve().parents[1]
baseline='fast_livo2_failsafe:healthy-gap-handoff-dev-20261014'
candidate='fast_livo2_failsafe:scan-descriptor-isolated-dev-20261015'
path='/ws/src/Fast_Livo2/src/step3_global_acquirer.cpp'
baseline_src=subprocess.check_output(['docker','run','--rm','--entrypoint','cat',baseline,path],text=True)
working=(root/'src/step3_global_acquirer.cpp').read_text()

def old_start(s):return s.index('double descriptorDistance(')
def new_start(s):return s.index('std::pair<double,int> bestDescriptorDistance(')
def end(s,start):return s.index('\n}\n\nclass Step3GlobalAcquirer',start)

replacement=working[new_start(working):end(working,new_start(working))]
old_segment=baseline_src[old_start(baseline_src):end(baseline_src,old_start(baseline_src))]
assert 'bestDescriptorDistance' in old_segment and 'descriptorDistance' in old_segment
assert 'q_norm2' in replacement and 'best_shift' in replacement
hybrid=baseline_src.replace(old_segment,replacement)
assert hybrid!=baseline_src and hybrid.count('bestDescriptorDistance(')==baseline_src.count('bestDescriptorDistance(')
diff=list(difflib.unified_diff(baseline_src.splitlines(),hybrid.splitlines(),n=1))
assert len([x for x in diff if x.startswith('@@')])==1,'Unexpected unrelated source changes'

context=root/'step3_global/isolated_descriptor_normcache_dev_20261015'
context.mkdir(parents=True,exist_ok=True)
(context/'step3_global_acquirer.cpp').write_text(hybrid)
(context/'Dockerfile').write_text(f'''FROM {baseline}
COPY step3_global_acquirer.cpp {path}
RUN /bin/bash -lc 'source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash && make -C /ws/build -j3 step3_global_acquirer'
''')
(context/'only_descriptor_diff.patch').write_text('\n'.join(diff)+'\n')
print(f'Isolated candidate: {candidate}, one hunk only, baseline={baseline}',flush=True)
subprocess.run(['docker','build','-t',candidate,str(context)],check=True)
