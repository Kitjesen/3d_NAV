"""Fix semantic_common colcon metadata on S100P."""
import paramiko
import time

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect('192.168.66.190', username='sunrise', password='sunrise', timeout=10)

pkg = 'semantic_common'
base = f'/opt/nav/install/{pkg}/share/{pkg}'

# Create hook directory
ssh.exec_command(f'mkdir -p {base}/hook')
time.sleep(0.5)

sftp = ssh.open_sftp()

# pythonpath.dsv
with sftp.file(f'{base}/hook/pythonpath.dsv', 'w') as f:
    f.write('prepend-non-duplicate;PYTHONPATH;lib/python3.10/site-packages\n')

# pythonpath.sh
pythonpath_sh = r"""# generated from colcon_python_setup_py
_colcon_python_setup_py_prepend_unique() {
  _v="$1"
  eval _vals="\$$_v"
  case ":$_vals:" in
    *":$2:"*) ;;
    *) eval "export $_v=\"$2\${_vals:+:\$_vals}\"" ;;
  esac
}
_colcon_python_setup_py_prepend_unique PYTHONPATH "$COLCON_CURRENT_PREFIX/lib/python3.10/site-packages"
"""
with sftp.file(f'{base}/hook/pythonpath.sh', 'w') as f:
    f.write(pythonpath_sh)

with sftp.file(f'{base}/hook/pythonpath.ps1', 'w') as f:
    f.write('')

# ament_prefix_path.dsv
with sftp.file(f'{base}/hook/ament_prefix_path.dsv', 'w') as f:
    f.write('prepend-non-duplicate;AMENT_PREFIX_PATH;\n')

# ament_prefix_path.sh
ament_sh = r"""# generated from colcon_core/shell/template/command_prefix.sh.em
_colcon_prepend_unique() {
  _v="$1"
  eval _vals="\$$_v"
  case ":$_vals:" in
    *":$2:"*) ;;
    *) eval "export $_v=\"$2\${_vals:+:\$_vals}\"" ;;
  esac
}
_colcon_prepend_unique AMENT_PREFIX_PATH "$COLCON_CURRENT_PREFIX"
"""
with sftp.file(f'{base}/hook/ament_prefix_path.sh', 'w') as f:
    f.write(ament_sh)

with sftp.file(f'{base}/hook/ament_prefix_path.ps1', 'w') as f:
    f.write('')

# package.dsv
dsv = f"""source;share/{pkg}/hook/pythonpath.ps1
source;share/{pkg}/hook/pythonpath.dsv
source;share/{pkg}/hook/pythonpath.sh
source;share/{pkg}/hook/ament_prefix_path.ps1
source;share/{pkg}/hook/ament_prefix_path.dsv
source;share/{pkg}/hook/ament_prefix_path.sh
"""
with sftp.file(f'{base}/package.dsv', 'w') as f:
    f.write(dsv)

# package.sh / package.bash / package.zsh
for ext in ['sh', 'bash', 'zsh']:
    with sftp.file(f'{base}/package.{ext}', 'w') as f:
        f.write(f'# generated from colcon_core/shell/template/package.{ext}\n')

sftp.close()
print('Created colcon metadata files for semantic_common')

# Verify import
time.sleep(1)
stdin, stdout, stderr = ssh.exec_command(
    'source /opt/ros/humble/setup.bash && source /opt/nav/install/setup.bash 2>/dev/null && '
    'python3 -c "import semantic_common; print(semantic_common.__file__)" 2>&1',
    timeout=10
)
print('Verify:', stdout.read().decode().strip())

# Restart nav-semantic
stdin, stdout, stderr = ssh.exec_command('sudo systemctl restart nav-semantic', timeout=15)
stdout.read()
print('Restarted nav-semantic')

time.sleep(8)
stdin, stdout, stderr = ssh.exec_command('systemctl is-active nav-semantic', timeout=5)
print('Status:', stdout.read().decode().strip())

stdin, stdout, stderr = ssh.exec_command(
    'journalctl -u nav-semantic --no-pager -n 10 2>/dev/null', timeout=10
)
print(stdout.read().decode('utf-8', errors='replace').strip()[-800:])

ssh.close()
