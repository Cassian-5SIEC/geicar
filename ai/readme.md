# Environment to develop AI models
This directory contains configurations and instructions to set up an environment for developing AI models. Follow the steps below to get started.
## Prerequisites
- Python 3.8 or higher
- pip (Python package installer)
- venv (Python virtual environment)

## Setting Up the Environment
1. Navigate to this directory and create a virtual environment:
```bash
python3 -m venv .venv
```
2. Activate the virtual environment:
```bash
source .venv/bin/activate
```
On windows, it is recommended to use Bash shell in VSCode terminal.

3. Install the required packages, make sure the virtual environment is activated:
```bash
pip install -r requirements.txt
```

## Adding New Dependencies
It is PROHIBITED to use `pip install <package-name>` directly. Instead, add your new dependency to `requirements.txt`, with optionnal version specifier, like this:
```
matplotlib==3.10.7
``` 

- Then run:
```bash
pip install -r requirements.txt
```
