IN distrobox enter -r aic_eval:
/entrypoint.sh ground_truth:=true start_aic_engine:=true gazebo_gui:=false


RUN IN SECOND UBUNTU TERMINAL UNDER: cd ~/ws_aic/src/aic
for i in $(seq 1 30); do
    echo "=== Run $i / 30 ==="
    python3 generate_config.py --trials 6 --seed $((i * 37 + RANDOM))
    pixi run ros2 run aic_model aic_model \
        --ros-args \
        -p use_sim_time:=true \
        -p policy:=aic_example_policies.ros.DataCollector
    echo "Run $i done: $(find ~/aic_perception_data/images -name '*.png' | wc -l) images"
    echo "Restart Terminal 1 then press Enter..."
    read
done

data will be in aic_perception_data in home/... root directory, zip and give back to anshul once done, run this on nvidia gpu please