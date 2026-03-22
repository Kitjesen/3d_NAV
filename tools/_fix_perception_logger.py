"""Fix printf-style logger calls in perception_node.py on S100P."""
import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect('192.168.66.190', username='sunrise', password='sunrise', timeout=10)

path = '/opt/nav/install/semantic_perception/lib/python3.10/site-packages/semantic_perception/perception_node.py'

sftp = ssh.open_sftp()
with sftp.file(path, 'r') as f:
    content = f.read().decode()

# Fix the broken KG logger section
# Find "kg_stats = self._knowledge_graph.get_stats()" and replace everything
# after it until the KG vocab comment
import re

# Remove the broken multi-line logger and replace with single line
lines = content.split('\n')
new_lines = []
skip_until_kg_vocab = False

for i, line in enumerate(lines):
    if skip_until_kg_vocab:
        if '用 KG 词汇表' in line or '# 用 KG' in line:
            skip_until_kg_vocab = False
            new_lines.append(line)
        # Skip broken lines
        continue

    if 'kg_stats = self._knowledge_graph.get_stats()' in line:
        new_lines.append(line)
        new_lines.append('            self.get_logger().info("KG injected: %d concepts, %d relations, %d safety constraints" % (kg_stats["total_concepts"], kg_stats["total_relations"], kg_stats["total_safety_constraints"]))')
        new_lines.append('')
        skip_until_kg_vocab = True
        continue

    new_lines.append(line)

content = '\n'.join(new_lines)

# Verify syntax
import ast
try:
    ast.parse(content)
    print('SYNTAX OK')
except SyntaxError as e:
    print(f'SYNTAX ERROR: {e}')
    ssh.close()
    exit(1)

with sftp.file(path, 'w') as f:
    f.write(content)

sftp.close()
print('File updated on S100P')
ssh.close()
