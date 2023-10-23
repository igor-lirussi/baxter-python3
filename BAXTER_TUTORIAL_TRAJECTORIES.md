# BAXTER TUTORIAL TRAJECTORIES 

## Record
Execute the recorder in the _baxter-python3_ interface folder. 

(In the Baxter's lab external computer is inside _Desktop/catkin_ws/src_, I suggest to do it from this pc, cause you can just visualize immediately and copy the recordings files without ssh) 

(If you ssh in the internal robot computer, it's inside _igor_ folder, remember to activate a ros environment before, es ```source ~/alper_workspace/activate_env.sh``` ) 

The options:
```
python example_cartesian_space_recorder.py --help
```
You can specify the arm, recording rate, recordings name, and if you want also the joint-space recorded.

Files will be saved in current directory unless otherwise specified.

## Visualize
Execute the visualizer inside the same _baxter-python3_ interface folder. 

(In the Baxter's lab external computer is inside the same folder _Desktop/catkin_ws/src_) 

(If you ssh in the internal robot computer, you can't visualize there the trajectories, once copied them in your pc, use the script from there)

It will show all cartesian trajectories.

```
python visualizer_cartesian_trajectories.py
```

## Playback
Execute the playback inside the same _baxter-python3_ interface folder. 

(In the Baxter's lab external computer is inside the same folder _Desktop/catkin_ws/src_) 

(If you ssh in the internal robot computer, it's inside _igor_ folder, remember to activate a ros environment before, es ```source ~/alper_workspace/activate_env.sh``` ) 

```
python example_cartesian_space_playback.py --help
```
