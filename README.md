# pepper_ros_team7
# Dependencies

Before you start using this project implemented in the ROS framework, make sure your system satisfies the following dependencies.

## Install RASA

```bash
sudo apt update
sudo apt install python3-pip
pip3 install pip
pip3 install rasa
pip3 install rasa[spacy]
pip3 install rasa[transformers]
```

### Install the NLP model

```
python3 -m spacy download en_core_web_md
```

### Install Duckling

```bash
cd workspace/src/rasa_ros/rasa-project/
sudo apt install libicu-dev
sudo apt install libpcre3-dev
wget -qO- https://get.haskellstack.org/ | sh 
git clone https://github.com/facebook/duckling
cd duckling
stack build
```

### Install inflect

```
pip3 install inflect
```

## Install Flask

```
pip3 install flask
```

# Run project

Execute the following commands to run the project.

## Build

```bash
cd workspace
chmod u+x src/*
catkin build
```

## Run 

Run each `roslaunch` command in individual shells.

```bash
cd workspace
source devel/setup.bash
roslaunch pepper_nodes pepper_bringup.launch 
```

```bash
cd workspace
source devel/setup.bash
roslaunch detector_pkg detector.launch 
```

```bash
cd workspace
source devel/setup.bash
roslaunch tablet_pkg tablet.launch
```

```bash
cd workspace
source devel/setup.bash
roslaunch ros_audio_pkg speech_recognition.launch 
```

```bash
cd workspace
source devel/setup.bash
roslaunch rasa_ros dialogue.xml
```

