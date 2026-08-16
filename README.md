# Quick setup

Linux / WSL / Git Bash
```bash
./setup.sh
source .venv/bin/activate
python main.py
```

Windows PowerShell
```powershell

# Or create a native Windows venv (uses Scripts\):
python -m venv .venv
. .\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
python main.py
```