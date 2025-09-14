## Setting up Docker

Download docker desktop on Windows machine

## Setting up WSL

Tip - if you already have WSL, you can use: ```lsb_release -a``` to see what version of Linux you're on. 

Open PowerShell or Command Prompt as Administrator:

. Search for "PowerShell" or "Command Prompt" in the Windows search bar.

· Right-click on the result and select "Run as administrator".

Execute the installation command:

· Type ```ws1 -- install``` and press Enter.

· You may need to confirm the installation by pressing 'y' when prompted.

For coding, I use VSCode with the WSL extension.

### Configure WSL to use the bridge:

Navigate to your Windows user profile folder by typing ```%UserProfile%``` in the File Explorer address bar.

Create or edit a file named .wslconfig and
Add the following content:
```
[wsl2]
networkingMode=bridged
vmSwitch=Default Switch
```
Restart WSL: In the same PowerShell admin window, run:

PowerShell

```wsl --shutdown```

Now, start your WSL Ubuntu terminal. It will reboot and connect to the new bridge.

```echo "export DISPLAY=:0" >> ~/.bashrc```

```echo "export LIBGL_ALWAYS_INDIRECT=0" >> ~/.bashrc```

```source ~/.bashrc```

## Running the Docker Image

```cd <top_project_folder>```

```docker-compose build windows_command```

```docker-compose up windows_command```