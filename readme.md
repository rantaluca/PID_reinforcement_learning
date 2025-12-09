## Installation du MATLAB Engine pour Python

### Prérequis
- MATLAB 2024/2025 installé
- Python 3 installé et accessible dans le terminal

### macOS et Linux

1. Ouvrir un terminal.
2. Se rendre dans le dossier :

cd /Applications/MATLAB_R20XXx.app/extern/engines/python

Il faut remplacer `R20XXx` par la version de MATLAB installée (par exemple, `R2024a`, `R2025a`).

3. Installer avec :

python3 setup.py install

### Windows

1. Ouvrir CMD ou PowerShell.
2. Se rendre dans le dossier :

cd "C:\Program Files\MATLAB\R20XXx\extern\engines\python"

(Il faut remplacer `R20XXx` par la version de MATLAB installée, par exemple, `R2024a`, `R2025a`).
3. Installer avec :

python setup.py install

### La simulation MATLAB peut maintenant s'exécuter dans le code Python 

### Autres (utilisateurs Windows)

1. Créer le dossier suivant : "C:\temp"

2. Dans le main, choisir le bon sys866_lib à importer
