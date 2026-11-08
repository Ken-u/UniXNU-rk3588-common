#!/usr/bin/env python3
"""Finds or fetches the sources of Rockchip's MPP that the encoder core compiles, and checks them against core/mpp.lock.

  fetch_mpp.py [CACHE_DIR]      prints the directory of a verified checkout

Order: $MPP (a checkout; its commit must be the locked one if it is a git repository), else CACHE_DIR/<commit> (downloaded from GitHub's codeload on the
first use), else download. Either way the files the core uses (core/mpp-files.txt and the headers of the directories in core/mpp-includes.txt) must hash to
the locked tree_sha256, so a moved tag or a changed mirror is noticed. `fetch_mpp.py --hash DIR` prints the hash of a tree (to make a lock).
"""
import hashlib
import io
import os
import subprocess
import sys
import tarfile
import urllib.request
from pathlib import Path

CORE = Path(__file__).resolve().parent


def read_lines(name):
    return [line.strip() for line in (CORE / name).read_text().splitlines() if line.strip() and not line.startswith('#')]


def read_lock():
    lock = dict(line.split('=', 1) for line in read_lines('mpp.lock'))
    for key in ('repo', 'commit', 'tree_sha256'):
        if key not in lock:
            sys.exit('fetch_mpp: core/mpp.lock has no %s' % key)
    return lock


def used_files(root):
    names = set(read_lines('mpp-files.txt'))
    for directory in read_lines('mpp-includes.txt'):
        folder = Path(root) / directory
        if folder.is_dir():
            names.update(str(path.relative_to(root)) for path in folder.iterdir() if path.is_file() and path.suffix == '.h')
    return sorted(names)


def tree_hash(root):
    digest = hashlib.sha256()
    for name in used_files(root):
        path = Path(root) / name
        if not path.is_file():
            sys.exit('fetch_mpp: %s is missing in %s' % (name, root))
        digest.update(('%s %s\n' % (name, hashlib.sha256(path.read_bytes()).hexdigest())).encode())
    return digest.hexdigest()


def download(lock, destination):
    url = lock['repo'].replace('https://github.com/', 'https://codeload.github.com/').rstrip('/') + '/tar.gz/' + lock['commit']
    print('fetch_mpp: downloading %s' % url, file=sys.stderr)
    with urllib.request.urlopen(url, timeout=120) as response:
        data = response.read()
    destination.mkdir(parents=True, exist_ok=True)
    with tarfile.open(fileobj=io.BytesIO(data), mode='r:gz') as archive:
        for member in archive.getmembers():
            parts = Path(member.name).parts
            if len(parts) < 2 or not (member.isfile() or member.isdir()):
                continue
            target = destination.joinpath(*parts[1:])
            if destination.resolve() not in target.resolve().parents and target.resolve() != destination.resolve():
                sys.exit('fetch_mpp: %s escapes the destination' % member.name)
            if member.isdir():
                target.mkdir(parents=True, exist_ok=True)
            else:
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(archive.extractfile(member).read())


def find(cache=None):
    lock = read_lock()
    given = os.environ.get('MPP')
    if given:
        root = Path(given)
        if (root / '.git').exists():
            have = subprocess.run(['git', '-C', str(root), 'rev-parse', 'HEAD'], stdout=subprocess.PIPE, text=True).stdout.strip()
            if have != lock['commit']:
                sys.exit('fetch_mpp: %s is at %s, core/mpp.lock wants %s' % (root, have, lock['commit']))
    else:
        root = Path(cache or Path.home() / '.cache' / 'unixnu-mpp') / lock['commit']
        if not (root / 'mpp').is_dir():
            part = root.with_name(root.name + '.part')
            if part.exists():
                subprocess.run(['rm', '-rf', str(part)], check=True)
            download(lock, part)
            if tree_hash(part) != lock['tree_sha256']:
                sys.exit('fetch_mpp: the download of %s does not match core/mpp.lock (tree_sha256 %s, locked %s)' % (lock['commit'], tree_hash(part), lock['tree_sha256']))
            part.rename(root)
    have = tree_hash(root)
    if have != lock['tree_sha256']:
        sys.exit('fetch_mpp: %s has tree_sha256 %s, core/mpp.lock wants %s' % (root, have, lock['tree_sha256']))
    return root


if __name__ == '__main__':
    if len(sys.argv) == 3 and sys.argv[1] == '--hash':
        print(tree_hash(sys.argv[2]))
    else:
        print(find(sys.argv[1] if len(sys.argv) > 1 else None))
