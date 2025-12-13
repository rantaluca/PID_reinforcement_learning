# Projet SYS866 : Contrôle PID par Apprentissage par Renforcement
## Auteurs : Céline Ngyuen, Erwan Audousset, Robert Antaluca, Romain Lemaire

Ce ReadMe décrit les étapes pour installer et exécuter notre projet de contrôle PID .

Nous vous conseillons d'executer ce projet sur un Mac ou Linux pour une meilleure compatibilité avec le MATLAB Engine python sur lequel repose la simulation.

## Installation du MATLAB Engine pour Python

### Prérequis
- MATLAB 2025 installé
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

4. Créer le dossier suivant à la racine  : "C:\temp"

### La simulation MATLAB peut maintenant s'exécuter dans le code Python 

## Essayer le code

Pour éxécuter un épisode avec l'une des politiques implémentées vous pouvez lancer au choix les scripts suivants dans le terminal à la racine du projet :

#### Politique P2: 

```sh
python3 main_p2.py
```

#### Politique AC (Actor-Critic):

```sh
python3 main_ac_inferance.py
````





