#!/usr/bin/env python3

import os
import subprocess
import shutil

current_branch = subprocess.run(
    ['git', 'rev-parse', '--abbrev-ref', 'HEAD'], stdout=subprocess.PIPE
).stdout.decode('utf-8')

if current_branch == 'cargo-only\n':
    print("Can't delete current branch")
    import sys
    sys.exit(1)

os.system('git branch -D cargo-only')
os.system('git checkout -b cargo-only')

with open('Cargo.toml') as f:
    cargo_toml = f.readlines()
with open('Cargo.toml', 'w') as f:
    for l in cargo_toml:
        if not l.startswith('crate-type'):
            f.write(l)

os.remove('api.json')
shutil.copyfile('../module/api.json', 'api.json')

os.system('git add Cargo.toml api.json')
os.system('git commit -m "Update cargo-only branch"')
os.system('git checkout %s' % current_branch)

print()
print('Run `git push -u origin cargo-only` to update the remote repository')
