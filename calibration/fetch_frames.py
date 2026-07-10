import sys, os, paramiko
host, remote, local = sys.argv[1], sys.argv[2], sys.argv[3]
os.makedirs(local, exist_ok=True)
c = paramiko.SSHClient(); c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(host, username="dexi", password="dexi", timeout=15, look_for_keys=False, allow_agent=False)
sftp = c.open_sftp()
files = [f for f in sftp.listdir(remote) if f.endswith(".png")]
for f in sorted(files):
    sftp.get(os.path.join(remote, f), os.path.join(local, f))
print(f"pulled {len(files)} frames -> {local}")
sftp.close(); c.close()
