# BAXTER TUTORIAL TRAJECTORIES 

The safe pipeline is *Record* -> *Visualize* -> *Playback*, this verifies your trajectories before going and trainig your model to generate new ones. 

## Record
Execute the recorder in this _baxter-python3_ project, clone the repo if you don't have it. 

I suggest to do it from the pc, without ssh into robot and cloning stuff there, you can esily mess up stuff, also from a pc you can visualize immediately and copy easily the recordings files to your usb.

(If you ssh in the internal robot computer, it's inside _igor_ folder, remember to activate a ros environment before, es ```source ~/alper_workspace/activate_env.sh``` ) 

Check the options:
You can specify the arm(s), recording rate, fixed duration, recordings name, and if you want also the joint-space recorded.
```
python example_trajectories_recorder.py --help
```

## Visualize
You should now have the trajectories in .CSV. 

Execute the visualizer inside the same folder, it has also some cool options to check. 

(If you ssh in the internal robot computer, you can't visualize there the trajectories, once copied them in your pc, use the script from there)

It will show all cartesian trajectories.

```
python visualizer_cartesian_trajectories.py
```

## Playback
Execute the playback code inside the same _baxter-python3_ interface folder, test it with the trajectories recorded and use this to play the new trajectories that you generated. 

(If you ssh in the internal robot computer, it's inside _igor_ folder, remember to activate a ros environment before, es ```source ~/alper_workspace/activate_env.sh``` ) 

```
python example_cartesian_space_playback.py --help
```
