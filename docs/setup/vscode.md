# EVC Dev Environment Setup (VS Code)

This guide walks you through setting up the EVC 2025 development environment using **VS Code Dev Containers**.

---

## Prerequisites

1. **Install Docker**
   - [Docker Desktop](https://docs.docker.com/desktop/setup/install/windows-install/) for Windows.
   - [Docker Desktop](https://docs.docker.com/desktop/setup/install/mac-install/) for MAC.
   - [Docker Engine](https://docs.docker.com/engine/install/) for Linux.
   - Verify:
     ```bash
     docker run --rm hello-world
     ```

2. **Install VS Code**
   - [Download VS Code](https://code.visualstudio.com/Download)

3. **Install Extensions**  
   - [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
   - [Python](https://www.python.org/downloads/release/python-31018/)

4. **Authenticate with GHCR**  
    To get your token, click [here](https://github.com/settings/tokens) and generate a new token.
   ```bash
   docker login ghcr.io -u <your-username> -p <your-github-pat>
5. **Clone the repo into a new project**  
    ```bash
   git clone https://github.com/EVC-Purdue/AutonomousKart
6. **Open with VSCode**  
    Open the repo and ensure you see the files. VSCode should automatically find `.devcontainers`
7.  **Reopen in container**  
    Press `CTRL+Shift+P` -> `Dev containers` -> `Reopen in Container`\
    VSCode should:
    - Create a new project
    - Pull the ghcr image
    - start the dev service
    - mount the repo at /ws
    - Run setup scripts
    - Install dependencies

