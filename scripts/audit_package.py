#!/usr/bin/env python3
"""Inspect files Git would publish, including size and accidental runtime data."""
from pathlib import Path
import re
import subprocess
ROOT=Path(__file__).resolve().parents[1]
listed=subprocess.check_output(['git','-C',str(ROOT),'ls-files','--cached','--others','--exclude-standard','-z']).decode().split('\0')
paths=[ROOT/p for p in set(listed) if p and (ROOT/p).is_file()]
for p in paths:
    relative=p.relative_to(ROOT)
    assert not any(part in relative.parts for part in ['.cache','.local','.venv','.migration','isaac_ws','node_modules','build','install']),relative
    assert p.suffix not in ['.bson','.ulg','.db3','.mcap','.bag','.pyc'],relative
    assert p.stat().st_size<50*1024**2,relative
    if p.stat().st_size<2*1024**2:
        data=p.read_bytes()
        patterns=[rb'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----',rb'gh[pousr]_[A-Za-z0-9]{30,}',rb'github_pat_[A-Za-z0-9_]{30,}']
        for pattern in patterns: assert not re.search(pattern,data),f'Potential credential: {relative}'
print(f'PACKAGE_AUDIT_OK: {len(paths)} files, {sum(p.stat().st_size for p in paths)/1024**2:.1f} MiB; no excluded runtime data or detected tokens')
