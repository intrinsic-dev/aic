#!/usr/bin/env python3
import os
import csv
import argparse
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def get_message_type(type_str):
    return get_message(type_str)

def process_interval(interval, threshold, bag_start_t):
    wrenches = interval['wrenches']
    if not wrenches:
        return {'avg_fz': 0.0, 'peak_fz': 0.0, 'violations': [], 'first_violation_start': float('inf')}
        
    start_t = interval['start']
    bias_wrenches = [fz for t, fz in wrenches if t <= start_t + 0.5]
    if bias_wrenches:
        bias = sum(bias_wrenches) / len(bias_wrenches)
    else:
        bias = wrenches[0][1]
        
    net_fzs = [(t, abs(fz - bias)) for t, fz in wrenches]
    
    overall_avg = sum(f for t, f in net_fzs) / len(net_fzs)
    overall_peak = max(f for t, f in net_fzs)
    
    violations = []
    current_block = []
    for t, f in net_fzs:
        if f > threshold:
            current_block.append((t, f))
        else:
            if current_block:
                dur = current_block[-1][0] - current_block[0][0]
                if dur > 1.0:
                    violations.append(current_block)
            current_block = []
            
    if current_block:
        dur = current_block[-1][0] - current_block[0][0]
        if dur > 1.0:
            violations.append(current_block)
            
    formatted_violations = []
    first_violation_start = float('inf')
    
    for block in violations:
        avg = sum(f for t, f in block) / len(block)
        rel_start = block[0][0] - bag_start_t
        rel_end = block[-1][0] - bag_start_t
        first_violation_start = min(first_violation_start, block[0][0])
        formatted_violations.append(f"{rel_start:.1f}s-{rel_end:.1f}s (Avg: {avg:.1f}N)")
        
    return {
        'avg_fz': overall_avg,
        'peak_fz': overall_peak,
        'violations': formatted_violations,
        'first_violation_start': first_violation_start
    }

def process_bag(bag_path):
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id='mcap'
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Failed to open bag {bag_path}: {e}")
        return []
    
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    
    wrench_topic = '/fts_broadcaster/wrench'
    rosout_topic = '/rosout'
    
    if wrench_topic not in type_map or rosout_topic not in type_map:
        print(f"Warning: Missing required topics in {bag_path}")
        return []
        
    wrench_msg_type = get_message_type(type_map[wrench_topic])
    rosout_msg_type = get_message_type(type_map[rosout_topic])
    
    switch_times = []
    tare_times = []
    wrenches = []
    
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        
        if topic == rosout_topic:
            msg = deserialize_message(data, rosout_msg_type)
            t = msg.stamp.sec + msg.stamp.nanosec * 1e-9
            if 'SwitchToAICController' in msg.msg:
                if not switch_times or t - switch_times[-1] > 1.0:
                    switch_times.append(t)
            elif 'TareForceTorqueSensorSkill' in msg.msg:
                if not tare_times or t - tare_times[-1] > 1.0:
                    tare_times.append(t)
                    
        elif topic == wrench_topic:
            msg = deserialize_message(data, wrench_msg_type)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            fz = msg.wrench.force.z
            wrenches.append((t, fz))
            
    if not wrenches:
        return []
        
    aic_switch_found = True
    start_times = switch_times
    if not start_times:
        start_times = tare_times
        aic_switch_found = False
        
    if not start_times:
        print(f"Warning: No SwitchToAICController or Tare commands found in {bag_path}")
        return []
        
    active_intervals = []
    for i, st in enumerate(start_times):
        end_t = start_times[i+1] if i + 1 < len(start_times) else wrenches[-1][0]
        active_intervals.append({'start': st, 'end': end_t})
        
    bag_start_t = wrenches[0][0]
    
    for interval in active_intervals:
        interval['wrenches'] = [(t, fz) for t, fz in wrenches if interval['start'] <= t <= interval['end'] + 0.1]
        
    results = []
    cable_index = 1
    for i in range(0, len(active_intervals), 2):
        sfp_interval = active_intervals[i]
        sc_interval = active_intervals[i+1] if i+1 < len(active_intervals) else None
        
        sfp_stats = process_interval(sfp_interval, 20.0, bag_start_t)
        sc_stats = process_interval(sc_interval, 30.0, bag_start_t) if sc_interval else None
        
        # Determine penalty attribution
        penalty_applied = False
        triggered_by = "N/A"
        
        sfp_triggered = len(sfp_stats['violations']) > 0
        sc_triggered = sc_stats and len(sc_stats['violations']) > 0
        
        if sfp_triggered and sc_triggered:
            penalty_applied = True
            triggered_by = "SFP" if sfp_stats['first_violation_start'] <= sc_stats['first_violation_start'] else "SC"
        elif sfp_triggered:
            penalty_applied = True
            triggered_by = "SFP"
        elif sc_triggered:
            penalty_applied = True
            triggered_by = "SC"
            
        cable_data = {
            'Cable Index': cable_index,
            'AIC Controller Switch Found': aic_switch_found,
            'Penalty Applied': penalty_applied,
            'Penalty Triggered By': triggered_by,
            'SFP Average Force (N)': f"{sfp_stats['avg_fz']:.2f}",
            'SC Average Force (N)': f"{sc_stats['avg_fz']:.2f}" if sc_stats else "N/A",
            'SFP Peak Force (N)': f"{sfp_stats['peak_fz']:.2f}",
            'SC Peak Force (N)': f"{sc_stats['peak_fz']:.2f}" if sc_stats else "N/A",
            'SFP Violations': "; ".join(sfp_stats['violations']),
            'SC Violations': "; ".join(sc_stats['violations']) if sc_stats else "N/A"
        }
        results.append(cable_data)
        cable_index += 1
        
    return results

def main():
    parser = argparse.ArgumentParser(description="Batch score offline bags")
    parser.add_argument('--folder', type=str, required=True, help="Root folder containing final_evals data")
    parser.add_argument('--team', type=str, default=None, help="Specific team name to process (default: all teams)")
    parser.add_argument('--eval-dir', type=str, default="final_eval", help="Directory name to filter valid bags (default: final_eval)")
    args = parser.parse_args()
    
    root_dir = os.path.abspath(args.folder)
    out_csv = os.path.join(root_dir, f"{args.team}_scoring_results.csv" if args.team else "scoring_results.csv")
    
    fieldnames = [
        'Team Name', 'Trial Name', 'Cable Index', 'AIC Controller Switch Found', 'Penalty Applied', 'Penalty Triggered By',
        'SFP Average Force (N)', 'SC Average Force (N)', 
        'SFP Peak Force (N)', 'SC Peak Force (N)', 
        'SFP Violations', 'SC Violations'
    ]
    
    with open(out_csv, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        
        for dirpath, dirnames, filenames in os.walk(root_dir):
            for file in filenames:
                if file.endswith(".mcap"):
                    bag_path = os.path.join(dirpath, file)
                    rel_path = os.path.relpath(bag_path, root_dir)
                    parts = rel_path.split(os.sep)
                    
                    if len(parts) >= 4:
                        team_name = parts[0]
                        trial_name = parts[-3]
                        cable_folder = parts[-2]
                    else:
                        team_name = "Unknown"
                        trial_name = "Unknown"
                        cable_folder = "Unknown"
                        
                    # Skip junk bags that aren't part of a final evaluation
                    if not any(args.eval_dir in p for p in parts):
                        continue
                        
                    if args.team and team_name != args.team:
                        continue
                        
                    print(f"Processing: {team_name} / {trial_name} -> {file}")
                    
                    cables = process_bag(bag_path)
                    for cable in cables:
                        row = {
                            'Team Name': team_name,
                            'Trial Name': trial_name
                        }
                        row.update(cable)
                        row['Cable Index'] = cable_folder
                        writer.writerow(row)
                        
    print(f"Scoring complete. Results saved to {out_csv}")

if __name__ == "__main__":
    main()
