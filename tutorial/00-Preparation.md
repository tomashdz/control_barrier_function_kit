# CBF Tutorial - Part 0: Preparration

## Python packages

You can install necesarry packages.

```bash
pip install -r requirements.txt
```

## [Optional] VScode setting

We recommend use VScode to follow the turorials.

### VScode extentions
You can install extentions below.

1. Jupyter
1. Markdown All in One

### VScode setting

You can set your code will run from "CWD" when it is python.
Here is setting.

Go, or make launch.json under the .vscode under your work-space.
You can add below.

```json
{
    "configurations": [
        {
            "name": "Python: Current File (Integrated Terminal)",
            "type": "python",
            "request": "launch",
            "program": "${file}",
            "console": "integratedTerminal",
            "cwd": "${fileDirname}"
        }
    ]
}
```

Even you have another configurations already, that does not matter.
You can add the setting additionally as the list.

The reference is https://newbedev.com/vscode-how-to-set-working-directory-for-debug