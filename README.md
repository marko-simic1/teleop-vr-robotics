## Docker setup guide

# Prerequisites
Before starting, ensure you have the following installed:
\- Git
\- Docker
To verify installation:
```bash
git --version
docker --version
```

# 1. Clone the repository
```bash
git clone https://github.com/marko-simic1/teleop-vr-robotics.git
cd teleop-vr-robotics
```
# 2. Switch branch
```bash
git checkout vedran/docker
```
# 3. Build the Docker Container
```bash
docker build -t dipl-proj .
```
# 4. Add Executable Permission to the Run Script
```bash
chmod +x run_docker.sh
```
# 5. Run docker (only first run)
```bash
./run_docker.sh
```
Every other time you start it with:
```bash
docker start -i dipl-proj-container
```
or, if you want bash shell inside the container
```bash
docker exec -it dipl-proj-container bash
```

# Next steps
After this, follow instructions from https://github.com/KhAlamdar11/unity-robotics-utils to connect ROS 2 and Unity