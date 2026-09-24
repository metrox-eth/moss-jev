"""Save a server credential outside the project, without displaying it."""
from getpass import getpass
from pathlib import Path
import os

if __name__=='__main__':
    key=getpass('OpenRouter API key (hidden): ').strip()
    if not key.startswith('sk-or-'):
        raise SystemExit('Expected an OpenRouter key. Nothing saved.')
    target=Path.home()/'.config/moss/openrouter.key'
    target.parent.mkdir(parents=True,exist_ok=True)
    fd=os.open(target,os.O_WRONLY|os.O_CREAT|os.O_TRUNC,0o600)
    os.fchmod(fd,0o600)
    with os.fdopen(fd,'w') as out:out.write(key+'\n')
    print('Credential saved outside the project. Restart the local MOSS server.')
