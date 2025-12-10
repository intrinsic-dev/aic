### Recording an episode

> [!NOTE]
> All commands should be run from the participant workspace.

Start the episode recorder node:

```bash
ros2 run rosetta episode_recorder --ros-args -p contract_path:=src/aic/aic_lerobot_tools/contracts/aic_phase0.yaml -p bag_base_dir:=episodes
```

Start recording:

```bash
ros2 action send_goal /record_episode rosetta_interfaces/action/RecordEpisode 'prompt: do stuff'
```

Finish recording:

```bash
ros2 service call /record_episode/cancel std_srvs/srv/Trigger '{}'
```

Convert the recording to leRobot dataset:

```bash
python -m bag_to_lerobot --contract src/aic/aic_lerobot_tools/contracts/aic_phase0.yaml --timestamp bag --out recording --bags episodes/*
```

> [!CAUTION]
> Converting actions are currently broken in `rosetta`, all actions will have 0 values.

### Contracts

A contract defines how to record and convert a ros bag into leRobot dataset.

#### decoders

`rosetta` require decoders to define how to convert a ros message. Search for `register_decoder` in `../../iblnkn/rosetta/rosetta/common/decoders.py` to see the list of messages supported by default.

Because `rosetta` does not support loading decoders dynamically yet, we need to wrap it to add new decoders. See [decoders.py](bag_to_lerobot/decoders.py).
