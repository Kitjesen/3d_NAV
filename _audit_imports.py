"""Audit: analyze package-level import dependencies."""
import ast, os
from collections import defaultdict

src = 'src'
pkg_files = defaultdict(list)

for root, dirs, files in os.walk(src):
    dirs[:] = [d for d in dirs if d not in ('tests','__pycache__','.omc','.pytest_cache','legacy','webrtc')]
    for f in files:
        if f.endswith('.py'):
            full = os.path.join(root, f)
            rel = os.path.relpath(full, src)
            parts = rel.replace(os.sep, '/').split('/')
            if len(parts) >= 1:
                pkg = parts[0]
                pkg_files[pkg].append(full)

syntax_errors = []
imports = defaultdict(set)
import_edges = []

for pkg, files in pkg_files.items():
    for fpath in files:
        try:
            with open(fpath, encoding='utf-8-sig') as f:
                tree = ast.parse(f.read())
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    for alias in node.names:
                        top = alias.name.split('.')[0]
                        if top in pkg_files and top != pkg:
                            imports[pkg].add(top)
                            import_edges.append((pkg, top, os.path.relpath(fpath, src)))
                elif isinstance(node, ast.ImportFrom):
                    if node.module and node.level == 0:
                        top = node.module.split('.')[0]
                        if top in pkg_files and top != pkg:
                            imports[pkg].add(top)
                            import_edges.append((pkg, top, os.path.relpath(fpath, src)))
        except SyntaxError as e:
            syntax_errors.append((os.path.relpath(fpath, src), str(e)))

print("=== Syntax errors (files skipped) ===")
for f, err in syntax_errors:
    print(f"  {f}: {err}")

print()
print("=== Package dependency graph (absolute imports only) ===")
for src_pkg in sorted(imports):
    deps = imports[src_pkg]
    if deps:
        print(f"  {src_pkg} -> {sorted(deps)}")
    else:
        print(f"  {src_pkg} -> (none)")

# Cycles via DFS
graph = dict(imports)
all_nodes = set(graph.keys())
for v in graph.values():
    all_nodes.update(v)

def find_simple_cycles(g):
    cycles = set()
    def dfs(start, current, visited, path):
        for neighbor in g.get(current, set()):
            if neighbor == start and len(path) > 1:
                norm = tuple(sorted(path + [start]))
                cycles.add(norm)
            elif neighbor not in visited:
                visited.add(neighbor)
                dfs(start, neighbor, visited, path + [neighbor])
    for n in all_nodes:
        dfs(n, n, {n}, [n])
    return cycles

cycles = find_simple_cycles(imports)
print()
print("=== Import Cycles Found ===")
if cycles:
    for c in sorted(cycles):
        print(f"  {' -> '.join(list(c) + [c[0]])}")
else:
    print("  None")

print()
print("=== Files creating memory-semantic cross-dependency ===")
for s, d, f in sorted(import_edges):
    if {s, d} == {'semantic', 'memory'}:
        print(f"  {f}: {s} -> {d}")

print()
print("=== Strongly-coupled packages (import from 2+ non-core siblings) ===")
for pkg in sorted(imports):
    non_core = imports[pkg] - {'core'}
    if len(non_core) >= 2:
        files_in = [(s,d,f) for s,d,f in import_edges if s==pkg and d in non_core]
        print(f"  {pkg} -> {non_core}")
        for s,d,f in files_in:
            print(f"      {f}")
